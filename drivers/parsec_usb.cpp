#include "parsec_usb.hpp"

#include <cctype>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace studica_driver {

namespace {

constexpr uint8_t kMagic0 = 0xA5;
constexpr uint8_t kMagic1 = 0x5A;
// magic u16, length u16, type, stream count, zones, flags
constexpr size_t kHeaderSize = 8;
constexpr uint8_t kTypeDistance = 1;
constexpr uint8_t kFlagDistance = 0x01;
// Largest frame the firmware sends: every per-zone array in verbose mode.
constexpr size_t kMaxFrameSize = kHeaderSize + ParsecUsb::kMaxZones * 17;
constexpr size_t kMaxTextLine = 256;
// Keeps commands from merging into one USB packet.
constexpr auto kCommandSpacing = std::chrono::milliseconds(5);
constexpr auto kReplyTimeout = std::chrono::milliseconds(300);
constexpr auto kConfigTimeout = std::chrono::milliseconds(500);
// RESOLUTION stops and restarts the sensor before it answers.
constexpr auto kResolutionTimeout = std::chrono::milliseconds(1000);

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

bool startsWith(const std::string& s, const char* prefix)
{
    return s.rfind(prefix, 0) == 0;
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
    tcflush(fd_, TCIOFLUSH);
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
    std::lock_guard<std::mutex> lock(write_mutex_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    const char* data = cmd.c_str();
    size_t left = cmd.size();
    while (left > 0) {
        const ssize_t n = write(fd_, data, left);
        if (n < 0) {
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                return false;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        data += n;
        left -= static_cast<size_t>(n);
    }
    // Wait for it to leave as one packet before the next command (not tcdrain(),
    // which has no timeout).
    int queued = 0;
    while (ioctl(fd_, TIOCOUTQ, &queued) == 0 && queued > 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(kCommandSpacing);
    return true;
}

bool ParsecUsb::ConfigureStreaming()
{
    if (fd_ < 0) {
        return false;
    }
    // A freshly powered Parsec is silent until it gets a command, so these also
    // wake it. Verbose mode, the boot default, adds 960 bytes of per-zone
    // diagnostics per frame that we don't use. Binary is the default, but set it
    // in case the device was left in JSON mode.
    const bool ok = SendCommand("VERBOSE,0") && SendCommand("DATAFORMAT,BIN");
    startReader();
    return ok;
}

ParsecUsb::Result ParsecUsb::command(const std::string& cmd, std::chrono::milliseconds timeout)
{
    if (fd_ < 0 || !running_) {
        return Result::NotOpen;
    }
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    {
        // Before sending: the reply can arrive before this thread waits for it.
        std::lock_guard<std::mutex> lock(reply_mutex_);
        awaiting_ = cmd;
        reply_ = Reply::None;
    }
    const bool sent = SendCommand(cmd);
    std::unique_lock<std::mutex> lock(reply_mutex_);
    const bool answered = sent && reply_cv_.wait_for(lock, timeout, [this] { return reply_ != Reply::None; });
    const Reply reply = reply_;
    awaiting_.clear();
    reply_ = Reply::None;
    if (!sent) {
        return Result::NotOpen;
    }
    if (!answered) {
        return Result::Timeout;
    }
    switch (reply) {
        case Reply::Ok: return Result::Ok;
        case Reply::Unknown: return Result::Unsupported;
        default: return Result::Rejected;
    }
}

ParsecUsb::Result ParsecUsb::SetOutputRate(int hz)
{
    return command("ODR," + std::to_string(hz), kReplyTimeout);
}

ParsecUsb::Result ParsecUsb::SetResolution(int zones)
{
    return command("RESOLUTION," + std::to_string(zones), kResolutionTimeout);
}

ParsecUsb::Result ParsecUsb::SetRangingFrequency(int hz)
{
    return command("FREQ," + std::to_string(hz), kReplyTimeout);
}

ParsecUsb::Result ParsecUsb::SetZoneMask(uint64_t mask)
{
    char cmd[32];
    std::snprintf(cmd, sizeof(cmd), "ZONEMASK,%016" PRIX64, mask);
    return command(cmd, kReplyTimeout);
}

const char* ParsecUsb::ResultString(Result result)
{
    switch (result) {
        case Result::Ok: return "ok";
        case Result::Rejected: return "rejected by sensor";
        case Result::Unsupported: return "not supported by this firmware";
        case Result::Timeout: return "no reply";
        default: return "port not open";
    }
}

bool ParsecUsb::RequestConfig(std::string* response_out)
{
    if (fd_ < 0 || response_out == nullptr || !running_) {
        return false;
    }
    uint64_t before;
    {
        std::lock_guard<std::mutex> lock(reply_mutex_);
        before = config_count_;
    }
    if (!SendCommand("GETCONFIG")) {
        return false;
    }
    std::unique_lock<std::mutex> lock(reply_mutex_);
    if (!reply_cv_.wait_for(lock, kConfigTimeout, [&] { return config_count_ != before; })) {
        return false;
    }
    *response_out = config_line_;
    return true;
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
    uint8_t chunk[512];
    while (running_) {
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
        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            fprintf(stderr, "Parsec USB: %s disconnected\n", port_.c_str());
            break;
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
        consumeRxBuffer();
    }
    running_ = false;  // commands now fail at once instead of timing out
}

// Splits the stream into binary distance frames (starting A5 5A) and the text
// between them (command replies and the GETCONFIG line), which can run straight
// on from frame bytes.
void ParsecUsb::consumeRxBuffer()
{
    size_t pos = 0;
    while (true) {
        while (pos + 1 < rx_buf_.size() && !(rx_buf_[pos] == kMagic0 && rx_buf_[pos + 1] == kMagic1)) {
            onTextByte(static_cast<char>(rx_buf_[pos++]));
        }
        if (pos + kHeaderSize > rx_buf_.size()) {
            break;  // need more bytes
        }
        const uint8_t* h = &rx_buf_[pos];
        // `length` counts from the type byte to the end of the payload.
        const size_t total = 4 + static_cast<size_t>(h[2] | (h[3] << 8));
        const uint8_t type = h[4];
        const uint8_t zones = h[6];
        const uint8_t flags = h[7];
        if (total < kHeaderSize || total > kMaxFrameSize || zones > kMaxZones) {
            onTextByte(static_cast<char>(rx_buf_[pos++]));  // not a real frame start
            continue;
        }
        if (pos + total > rx_buf_.size()) {
            break;
        }
        if (type == kTypeDistance && (flags & kFlagDistance) && total >= kHeaderSize + 2u * zones) {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_seq_ = h[5];
            latest_zones_ = zones;
            for (int i = 0; i < kMaxZones; ++i) {
                latest_fdist_[static_cast<size_t>(i)] =
                    i < zones ? static_cast<int16_t>(h[kHeaderSize + 2 * i] | (h[kHeaderSize + 2 * i + 1] << 8))
                              : static_cast<int16_t>(-1);
            }
            has_frame_ = true;
        }
        pos += total;
    }

    rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<std::ptrdiff_t>(pos));
    if (rx_buf_.size() > 4 * kMaxFrameSize) {
        rx_buf_.clear();  // never resynchronised; start over
    }
}

void ParsecUsb::onTextByte(char c)
{
    if (c == '\n' || c == '\r') {
        if (!text_line_.empty()) {
            onLine(text_line_);
        }
        text_line_.clear();
    } else if (text_line_.size() < kMaxTextLine) {
        text_line_.push_back(c);
    }
}

void ParsecUsb::onLine(const std::string& line)
{
    std::lock_guard<std::mutex> lock(reply_mutex_);
    // GETCONFIG: "SN: <serial>   FV: 1.0.4   ...   CAN_ID: 5   ZM: 0x..." (one line).
    const size_t sn = line.find("SN:");
    if (sn != std::string::npos) {
        config_line_ = line.substr(sn);
        ++config_count_;
        reply_cv_.notify_all();
        return;
    }
    const Reply reply = classifyReply(line);
    if (reply == Reply::None || awaiting_.empty() || reply_ != Reply::None || !isReplyTo(awaiting_, line)) {
        return;
    }
    reply_ = reply;
    reply_cv_.notify_all();
}

// Wording verified on firmware 1.0.3: "... set to ..." is OK; "Invalid ...",
// "... fail" and "... requires ..." are errors.
ParsecUsb::Reply ParsecUsb::classifyReply(const std::string& line)
{
    if (startsWith(line, "Unknown command")) {
        return Reply::Unknown;
    }
    if (startsWith(line, "Invalid") || line.find(" fail") != std::string::npos ||
        line.find(" requires ") != std::string::npos) {
        return Reply::Error;
    }
    if (line.find(" set to ") != std::string::npos) {
        return Reply::Ok;
    }
    return Reply::None;
}

bool ParsecUsb::isReplyTo(const std::string& cmd, const std::string& line)
{
    const std::string name = cmd.substr(0, cmd.find(','));
    // Most replies name the command ("Invalid FREQ. ...", "ODR and frequency set to 5 Hz").
    auto is_word = [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; };
    for (size_t at = line.find(name); at != std::string::npos; at = line.find(name, at + 1)) {
        const size_t end = at + name.size();
        if ((at == 0 || !is_word(line[at - 1])) && (end == line.size() || !is_word(line[end]))) {
            return true;
        }
    }
    // These don't (firmware usb_cmd.c), so match them by wording.
    // "Set frequency fail" answers both FREQ and ODR.
    struct Wording { const char* name; const char* phrase; };
    static const Wording kWordings[] = {
        {"FREQ", "Frequency set to"},          {"FREQ", "Set frequency fail"},
        {"ODR", "Set frequency fail"},         {"RESOLUTION", "Resolution set to"},
        {"RESOLUTION", "Set resolution fail"}, {"ZONEMASK", "Zone mask set to"},
    };
    for (const auto& w : kWordings) {
        if (name == w.name && line.find(w.phrase) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // namespace studica_driver
