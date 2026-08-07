#include "colore_usb.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <sstream>
#include <vector>

namespace studica_driver {

namespace {

// Uppercases a string (e.g. for COLORFORMAT command).
std::string to_upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

// Trims leading/trailing whitespace from a string
std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

// Splits comma-separated values, trimming whitespace (e.g. for SRGB: 255, 0, 128, outputs ["255", "0", "128"]).
std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) out.push_back(trim(tok));
    return out;
}

// Telemetry lines the firmware streams continuously after COLORFORMAT.
bool is_telemetry_line(const std::string& line) {
    return line.rfind("XYZ:", 0) == 0
        || line.rfind("SRGB:", 0) == 0
        || line.rfind("HEX:", 0) == 0
        || line.rfind("RGB:", 0) == 0
        || line.rfind("RAW:", 0) == 0
        || line.rfind("DIST:", 0) == 0;
}

// Lines that belong to a GETCONFIG dump (see firmware usb_cmd.c CMD_GETCONFIG).
bool is_getconfig_line(const std::string& line) {
    return line.rfind("SN:", 0) == 0
        || line.rfind("BRIGHTNESS:", 0) == 0
        || line.rfind("CALIB:", 0) == 0
        || line.rfind("STATUSLED:", 0) == 0
        || line.rfind("SAMPLETIME:", 0) == 0
        || line.rfind("MEASMODE:", 0) == 0
        || line.rfind("GAIN:", 0) == 0;
}

}  // namespace

// Constructor opens the port and starts the reader thread.
ColoreUsb::ColoreUsb(const std::string& port, int baud) : port_(port) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); fd_ = -1; return; }
    cfmakeraw(&tty);
    speed_t spd = (baud == 9600) ? B9600 : B115200;
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); fd_ = -1; return; }

    startReader();
}

// Destructor stops the reader thread and closes the port.
ColoreUsb::~ColoreUsb() {
    stopReader();
    if (fd_ >= 0) ::close(fd_);
}

// Starts the reader thread. 
void ColoreUsb::startReader() {
    running_ = true;
    reader_ = std::thread(&ColoreUsb::readerLoop, this);
}

// Stops the reader thread.
void ColoreUsb::stopReader() {
    running_ = false;
    if (reader_.joinable()) reader_.join();
    line_buf_.clear();
}

// Sends a command to the device.
bool ColoreUsb::SendCommand(const std::string& cmd) {
    if (fd_ < 0) return false;
    std::string out = cmd + "\r\n";
    ssize_t n = ::write(fd_, out.data(), out.size());
    return n == static_cast<ssize_t>(out.size());
}

// Configures the streaming of data from the device.
bool ColoreUsb::ConfigureStreaming(const std::string& color_format, int sample_ms) {
    if (fd_ < 0) return false;
    bool ok = SendCommand("COLORFORMAT," + to_upper(color_format));
    if (sample_ms > 0) ok = SendCommand("SAMPLETIME," + std::to_string(sample_ms)) && ok;
    return ok;
}

// Requests the configuration of the device.
// Pause the reader, drain stale samples, then collect only GETCONFIG lines.
bool ColoreUsb::RequestConfig(std::string* response_out) {
    if (fd_ < 0) return false;

    const bool was_running = running_;
    if (was_running) stopReader();

    // Discard buffered telemetry so GETCONFIG is not buried under XYZ backlog.
    {
        auto drain_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
        std::string line;
        while (std::chrono::steady_clock::now() < drain_end) {
            if (!readLineBlocking(&line, 20)) break;
        }
        // Drop any partial line left in the OS buffer mid-frame.
        tcflush(fd_, TCIFLUSH);
    }

    if (!SendCommand("GETCONFIG")) {
        if (was_running) startReader();
        return false;
    }

    std::string acc;
    std::string line;
    bool saw_sn = false;
    auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < t_end) {
        if (!readLineBlocking(&line, 80)) continue;
        if (line.empty() || is_telemetry_line(line)) continue;
        if (!is_getconfig_line(line)) continue;
        acc += line + "\n";
        if (line.rfind("SN:", 0) == 0) saw_sn = true;
        if (line.rfind("GAIN:", 0) == 0) break;  // last GETCONFIG line
    }

    if (was_running) startReader();

    if (!saw_sn || acc.empty()) return false;
    if (response_out) *response_out = trim(acc);
    return true;
}

// Reads the latest sample from the device.
bool ColoreUsb::ReadLatest(Sample* out) {
    std::lock_guard<std::mutex> lk(sample_mutex_);
    if (!has_sample_ || out == nullptr) return false;
    *out = latest_;
    return true;
}

// Reads one '\n'-terminated line from a small private buffer (used by RequestConfig
// only; the steady-state stream is consumed by readerLoop()).
bool ColoreUsb::readLineBlocking(std::string* line_out, int timeout_ms) {
    auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    char c;
    std::string acc;
    while (std::chrono::steady_clock::now() < t_end) {
        ssize_t n = ::read(fd_, &c, 1);
        if (n == 1) {
            if (c == '\n') { *line_out = trim(acc); return true; }
            if (c != '\r') acc += c;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    *line_out = trim(acc);
    return !acc.empty();
}

// The reader thread continuously reads from the USB port and parses lines.
void ColoreUsb::readerLoop() {
    char buf[256];
    while (running_) {
        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                char c = buf[i];
                if (c == '\n') { parseLine(trim(line_buf_)); line_buf_.clear(); }
                else if (c != '\r') line_buf_ += c;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

// Parses a single line of text from the device, updating the latest sample.
void ColoreUsb::parseLine(const std::string& raw_line) {
    if (raw_line.empty()) return;
    std::string line = raw_line;

    Sample s;
    {
        std::lock_guard<std::mutex> lk(sample_mutex_);
        s = latest_;  // carry forward fields not in this line
    }

    // Optional "DIST: <mm>  " prefix (smart-measurement mode)
    if (line.rfind("DIST:", 0) == 0) {
        size_t two_spaces = line.find("  ");
        std::string dist = line.substr(5, (two_spaces == std::string::npos ? line.size() : two_spaces) - 5);
        s.has_dist = true;
        s.dist_mm = std::strtof(trim(dist).c_str(), nullptr);
        if (two_spaces == std::string::npos) { /* dist-only line */ }
        else line = trim(line.substr(two_spaces + 2));
    }

    auto after_colon = [&](const std::string& l) {
        size_t c = l.find(':');
        return (c == std::string::npos) ? std::string() : trim(l.substr(c + 1));
    };

    bool got = false;
    if (line.rfind("SRGB:", 0) == 0) {
        auto v = split_csv(after_colon(line));
        if (v.size() >= 3) {
            s.has_srgb = true;
            s.r = static_cast<uint8_t>(std::strtol(v[0].c_str(), nullptr, 10));
            s.g = static_cast<uint8_t>(std::strtol(v[1].c_str(), nullptr, 10));
            s.b = static_cast<uint8_t>(std::strtol(v[2].c_str(), nullptr, 10));
            got = true;
        }
    } else if (line.rfind("HEX:", 0) == 0) {
        s.has_hex = true;
        s.hex = after_colon(line);
        got = true;
    } else if (line.rfind("XYZ:", 0) == 0) {
        auto v = split_csv(after_colon(line));
        if (v.size() >= 3) {
            s.has_xyz = true;
            s.x = std::strtof(v[0].c_str(), nullptr);
            s.y = std::strtof(v[1].c_str(), nullptr);
            s.z = std::strtof(v[2].c_str(), nullptr);
            got = true;
        }
    } else if (line.rfind("RGB:", 0) == 0) {
        auto v = split_csv(after_colon(line));
        if (v.size() >= 3) {
            s.has_tristim = true;
            s.fz  = static_cast<uint16_t>(std::strtol(v[0].c_str(), nullptr, 10));
            s.fy  = static_cast<uint16_t>(std::strtol(v[1].c_str(), nullptr, 10));
            s.fxl = static_cast<uint16_t>(std::strtol(v[2].c_str(), nullptr, 10));
            got = true;
        }
    }

    if (!got) return;  // IDENTITY / GETCONFIG / unknown lines: ignore here

    std::lock_guard<std::mutex> lk(sample_mutex_);
    s.seq = ++seq_counter_;
    latest_ = s;
    has_sample_ = true;
}

}  // namespace studica_driver
