#include "colore_usb.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <vector>

namespace studica_driver {

namespace {

// Keeps commands from merging into one USB packet.
constexpr auto kCommandSpacing = std::chrono::milliseconds(5);
constexpr auto kConfigTimeout = std::chrono::milliseconds(500);

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

bool starts_with(const std::string& s, const char* prefix) {
    return s.rfind(prefix, 0) == 0;
}

// Lines that belong to a GETCONFIG dump (see firmware usb_cmd.c CMD_GETCONFIG).
// It has no OK line; GAIN: is the last one.
bool is_getconfig_line(const std::string& line) {
    return starts_with(line, "SN:")
        || starts_with(line, "BRIGHTNESS:")
        || starts_with(line, "CALIB:")
        || starts_with(line, "STATUSLED:")
        || starts_with(line, "SAMPLETIME:")
        || starts_with(line, "MEASMODE:")
        || starts_with(line, "GAIN:");
}

// Replies: "SAMPLETIME,100 OK", "MEASMODE OFF OK", "ERR usage: ...", "Unknown command: ...".
bool ends_with_ok(const std::string& line) {
    return line.size() >= 3 && line.compare(line.size() - 3, 3, " OK") == 0;
}

// True if `line` contains the command's name (the part before its first comma) as a whole word.
bool mentions_command(const std::string& line, const std::string& cmd) {
    const std::string name = cmd.substr(0, cmd.find(','));
    if (name.empty()) return false;
    auto is_word = [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; };
    for (size_t at = line.find(name); at != std::string::npos; at = line.find(name, at + 1)) {
        const size_t end = at + name.size();
        if ((at == 0 || !is_word(line[at - 1])) && (end == line.size() || !is_word(line[end])))
            return true;
    }
    return false;
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
    tcflush(fd_, TCIOFLUSH);

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

// Sends one command as its own USB packet. The firmware takes each packet as a
// whole command, so there is no terminator; wait for it to leave before the next.
bool ColoreUsb::SendCommand(const std::string& cmd) {
    if (fd_ < 0) return false;
    std::lock_guard<std::mutex> lk(write_mutex_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    const char* data = cmd.data();
    size_t left = cmd.size();
    while (left > 0) {
        ssize_t n = ::write(fd_, data, left);
        if (n < 0) {
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) return false;
            if (std::chrono::steady_clock::now() >= deadline) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        data += n;
        left -= static_cast<size_t>(n);
    }
    int queued = 0;
    while (ioctl(fd_, TIOCOUTQ, &queued) == 0 && queued > 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(kCommandSpacing);
    return true;
}

// Sends a command and waits for the reply that names it.
ColoreUsb::Result ColoreUsb::command(const std::string& cmd, std::chrono::milliseconds timeout) {
    if (fd_ < 0 || !running_) return Result::NotOpen;
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    {
        // Before sending: the reply can arrive before this thread waits for it.
        std::lock_guard<std::mutex> lk(reply_mutex_);
        awaiting_ = cmd;
        reply_ = Reply::None;
    }
    const bool sent = SendCommand(cmd);
    std::unique_lock<std::mutex> lk(reply_mutex_);
    const bool answered = sent && reply_cv_.wait_for(lk, timeout, [this] { return reply_ != Reply::None; });
    const Reply reply = reply_;
    awaiting_.clear();
    reply_ = Reply::None;
    if (!sent) return Result::NotOpen;
    if (!answered) return Result::Timeout;
    switch (reply) {
        case Reply::Ok: return Result::Ok;
        case Reply::Unknown: return Result::Unsupported;
        default: return Result::Rejected;
    }
}

// Configures the streaming of data from the device. The first command also wakes it.
ColoreUsb::Result ColoreUsb::ConfigureStreaming(const std::string& color_format, int sample_ms) {
    Result result = command("COLORFORMAT," + to_upper(color_format));
    if (result == Result::Ok && sample_ms > 0) result = SetSampleTime(sample_ms);
    return result;
}

ColoreUsb::Result ColoreUsb::SetSampleTime(int ms) {
    return command("SAMPLETIME," + std::to_string(ms));
}

ColoreUsb::Result ColoreUsb::SetBrightness(int percent) {
    return command("BRIGHTNESS," + std::to_string(percent));
}

ColoreUsb::Result ColoreUsb::SetStatusLed(bool enable, int percent) {
    return command(std::string("STATUSLED,") + (enable ? "1," : "0,") + std::to_string(percent));
}

ColoreUsb::Result ColoreUsb::SetMeasureModeOff() {
    return command("MEASMODE,OFF");
}

ColoreUsb::Result ColoreUsb::SetMeasureModeFixed(float z_mm) {
    // "MEASMODE,<distance mm>,<ambient 0|1>"; ambient compensation off.
    char cmd[48];
    std::snprintf(cmd, sizeof(cmd), "MEASMODE,%g,0", static_cast<double>(z_mm));
    return command(cmd);
}

const char* ColoreUsb::ResultString(Result result) {
    switch (result) {
        case Result::Ok: return "ok";
        case Result::Rejected: return "rejected by sensor";
        case Result::Unsupported: return "not supported by this firmware";
        case Result::Timeout: return "no reply";
        default: return "port not open";
    }
}

// Requests the configuration of the device; the reader thread collects the reply.
bool ColoreUsb::RequestConfig(std::string* response_out) {
    if (fd_ < 0 || !running_) return false;
    uint64_t before;
    {
        std::lock_guard<std::mutex> lk(reply_mutex_);
        before = config_count_;
    }
    if (!SendCommand("GETCONFIG")) return false;
    std::unique_lock<std::mutex> lk(reply_mutex_);
    if (!reply_cv_.wait_for(lk, kConfigTimeout, [&] { return config_count_ != before; })) return false;
    if (response_out) *response_out = config_done_;
    return true;
}

// Reads the latest sample from the device.
bool ColoreUsb::ReadLatest(Sample* out) {
    std::lock_guard<std::mutex> lk(sample_mutex_);
    if (!has_sample_ || out == nullptr) return false;
    *out = latest_;
    return true;
}

// The reader thread continuously reads from the USB port and parses lines.
void ColoreUsb::readerLoop() {
    char buf[256];
    while (running_) {
        pollfd pfd{fd_, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 50);
        if (pr < 0 && errno != EINTR) break;
        if (pr <= 0) continue;
        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            std::fprintf(stderr, "Colore USB: %s disconnected\n", port_.c_str());
            break;
        }
        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) break;
        for (ssize_t i = 0; i < n; ++i) {
            char c = buf[i];
            if (c == '\n' || c == '\r') {
                const std::string line = trim(line_buf_);
                line_buf_.clear();
                if (!line.empty() && !onTextLine(line)) parseLine(line);
            } else if (line_buf_.size() < 1024) {
                line_buf_ += c;
            }
        }
    }
    running_ = false;  // commands now fail at once instead of timing out
}

// Handles command replies and GETCONFIG lines. Returns true if the line was one.
bool ColoreUsb::onTextLine(const std::string& line) {
    std::lock_guard<std::mutex> lk(reply_mutex_);
    if (is_getconfig_line(line)) {
        if (starts_with(line, "SN:")) config_acc_.clear();
        if (!config_acc_.empty()) config_acc_ += "\n";
        config_acc_ += line;
        if (starts_with(line, "GAIN:")) {
            config_done_ = config_acc_;
            config_acc_.clear();
            ++config_count_;
            reply_cv_.notify_all();
        }
        return true;
    }

    Reply reply = Reply::None;
    if (starts_with(line, "Unknown command")) reply = Reply::Unknown;
    else if (starts_with(line, "ERR")) reply = Reply::Error;
    else if (ends_with_ok(line) || line.find(" OK ") != std::string::npos) reply = Reply::Ok;
    if (reply == Reply::None) return false;

    // Replies to anything but the awaited command are dropped, as is a second one.
    if (!awaiting_.empty() && reply_ == Reply::None && mentions_command(line, awaiting_)) {
        reply_ = reply;
        reply_cv_.notify_all();
    }
    return true;
}

// Parses a single line of text from the device, updating the latest sample.
void ColoreUsb::parseLine(const std::string& raw_line) {
    if (raw_line.empty()) return;
    const std::string& line = raw_line;

    Sample s;
    {
        std::lock_guard<std::mutex> lk(sample_mutex_);
        s = latest_;  // carry forward fields not in this line
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

    if (!got) return;  // IDENTITY / unknown lines: ignore here

    std::lock_guard<std::mutex> lk(sample_mutex_);
    s.seq = ++seq_counter_;
    latest_ = s;
    has_sample_ = true;
}

}  // namespace studica_driver
