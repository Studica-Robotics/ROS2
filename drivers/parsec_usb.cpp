#include "parsec_usb.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace studica_driver {

namespace {

speed_t baudToConstant(int baud)
{
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200;
    }
}

}  // namespace

ParsecUsb::ParsecUsb(const std::string& port, int baud)
    : port_(port)
{
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close(fd_);
        fd_ = -1;
        return;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baudToConstant(baud));
    cfsetospeed(&tty, baudToConstant(baud));
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close(fd_);
        fd_ = -1;
        return;
    }
}

ParsecUsb::~ParsecUsb()
{
    stopReader();
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

void ParsecUsb::startReader()
{
    if (fd_ < 0 || running_) {
        return;
    }
    running_ = true;
    reader_ = std::thread(&ParsecUsb::readerLoop, this);
}

void ParsecUsb::stopReader()
{
    running_ = false;
    if (reader_.joinable()) {
        reader_.join();
    }
}

bool ParsecUsb::SendCommand(const std::string& cmd)
{
    if (fd_ < 0) {
        return false;
    }
    std::string line = cmd;
    if (line.empty() || line.back() != '\n') {
        line += "\r\n";
    }
    const char* data = line.c_str();
    size_t left = line.size();
    while (left > 0) {
        const ssize_t n = write(fd_, data, left);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return false;
        }
        data += n;
        left -= static_cast<size_t>(n);
    }
    return true;
}

bool ParsecUsb::ConfigureStreaming(int odr_hz)
{
    if (fd_ < 0) {
        return false;
    }
    if (!SendCommand("GETCONFIG")) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    std::string junk;
    while (readLine(&junk, 50)) {
        // drain GETCONFIG text response
    }

    if (!SendCommand("VERBOSE,0")) {
        return false;
    }
    if (!SendCommand("DATAFORMAT,BIN")) {
        return false;
    }
    if (odr_hz > 0) {
        SendCommand("ODR," + std::to_string(odr_hz));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (readLine(&junk, 20)) {
    }
    startReader();
    return true;
}

bool ParsecUsb::RequestConfig(std::string* response_out)
{
    if (fd_ < 0 || response_out == nullptr) {
        return false;
    }
    response_out->clear();

    const bool was_running = running_;
    if (was_running) {
        stopReader();
    }

    const bool ok = SendCommand("GETCONFIG") && readLine(response_out, 500);

    if (was_running) {
        startReader();
    }
    return ok;
}

int ParsecUsb::ReadLatestFdist(uint8_t* seq_out, uint8_t* zones_out, int16_t* fdist, int max_zones)
{
    if (fdist == nullptr || max_zones <= 0) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!has_frame_) {
        return 0;
    }

    const int n = (latest_zones_ < static_cast<uint8_t>(max_zones)) ? latest_zones_ : max_zones;
    for (int i = 0; i < n; ++i) {
        fdist[i] = latest_fdist_[static_cast<size_t>(i)];
    }
    if (seq_out) {
        *seq_out = latest_seq_;
    }
    if (zones_out) {
        *zones_out = latest_zones_;
    }
    return n;
}

void ParsecUsb::readerLoop()
{
    uint8_t chunk[256];
    while (running_) {
        if (fd_ < 0) {
            break;
        }

        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int pr = poll(&pfd, 1, 50);
        if (pr < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (pr == 0) {
            continue;
        }

        const ssize_t n = read(fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            break;
        }
        if (n == 0) {
            continue;
        }

        rx_buf_.insert(rx_buf_.end(), chunk, chunk + n);
        if (rx_buf_.size() > 8192) {
            rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - 4096);
        }
        consumeRxBuffer();
    }
}

void ParsecUsb::consumeRxBuffer()
{
    while (rx_buf_.size() >= sizeof(FrameHdr)) {
        size_t start = 0;
        bool found = false;
        for (size_t i = 0; i + 1 < rx_buf_.size(); ++i) {
            if (rx_buf_[i] == 0xA5 && rx_buf_[i + 1] == 0x5A) {
                start = i;
                found = true;
                break;
            }
        }
        if (!found) {
            if (rx_buf_.size() > 1) {
                rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - 1);
            }
            return;
        }
        if (start > 0) {
            rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<std::ptrdiff_t>(start));
        }
        if (rx_buf_.size() < sizeof(FrameHdr)) {
            return;
        }

        size_t frame_len = 0;
        if (!tryParseFrame(0, &frame_len)) {
            return;
        }
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<std::ptrdiff_t>(frame_len));
    }
}

bool ParsecUsb::tryParseFrame(size_t offset, size_t* frame_len_out)
{
    if (rx_buf_.size() < offset + sizeof(FrameHdr)) {
        return false;
    }

    FrameHdr hdr{};
    std::memcpy(&hdr, rx_buf_.data() + offset, sizeof(hdr));
    if (hdr.magic != kFrameMagic) {
        rx_buf_.erase(rx_buf_.begin());
        return false;
    }

    const size_t total = 4u + static_cast<size_t>(hdr.length);
    if (rx_buf_.size() < offset + total) {
        return false;
    }

    if (hdr.type != 1) {
        *frame_len_out = total;
        return true;
    }

    const uint8_t* payload = rx_buf_.data() + offset + sizeof(FrameHdr);
    const size_t payload_len = total - sizeof(FrameHdr);

    if ((hdr.flags & 0x01) == 0) {
        *frame_len_out = total;
        return true;
    }

    const size_t fdist_bytes = static_cast<size_t>(hdr.zones) * sizeof(int16_t);
    if (payload_len < fdist_bytes) {
        *frame_len_out = total;
        return true;
    }

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_seq_ = hdr.streamcount;
    latest_zones_ = hdr.zones;
    for (uint8_t i = 0; i < hdr.zones && i < kMaxZones; ++i) {
        int16_t v = 0;
        std::memcpy(&v, payload + static_cast<size_t>(i) * sizeof(int16_t), sizeof(v));
        latest_fdist_[i] = v;
    }
    for (uint8_t i = hdr.zones; i < kMaxZones; ++i) {
        latest_fdist_[i] = -1;
    }
    has_frame_ = true;

    *frame_len_out = total;
    return true;
}

bool ParsecUsb::readLine(std::string* line_out, int timeout_ms)
{
    if (fd_ < 0 || line_out == nullptr) {
        return false;
    }

    line_out->clear();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int wait_ms = 20;
        const int pr = poll(&pfd, 1, wait_ms);
        if (pr <= 0) {
            if (!line_out->empty()) {
                break;
            }
            continue;
        }

        char c = 0;
        const ssize_t n = read(fd_, &c, 1);
        if (n <= 0) {
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (!line_out->empty()) {
                return true;
            }
            continue;
        }
        line_out->push_back(c);
    }

    return !line_out->empty();
}

}  // namespace studica_driver
