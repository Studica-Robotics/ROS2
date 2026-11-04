#include "navx3_usb.hpp"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// DATA line format and scales from StudicaLib (NavxProtocol.cpp), verified on
// NavX3 firmware: "DATA: TS:[us] YAW:[mdeg] PITCH_ROLL:[mdeg,mdeg] ANGLE:[mdeg]
// QUAT:[w,x,y,z] QUAT9D:[...] GYRO:[x,y,z] ACCEL:[x,y,z] MAG:[x,y,z(,norm)]
// BIAS:[x,y,z] REST:n MAGDIST:n TEMP:[cdeg]". Replies can splice into a data
// line, so keys are searched for anywhere they start a word.

namespace studica_driver {

namespace {

constexpr double kQuatScale = 1000000.0;
constexpr double kGyroScale = 10000.0;
constexpr double kAccelScale = 1000.0;
constexpr double kMagScale = 1000.0;
constexpr double kAngleScale = 1000.0;
constexpr double kBiasScale = 1000000.0;
constexpr double kTempScale = 100.0;

// Keeps commands from merging into one USB packet.
constexpr auto kCommandSpacing = std::chrono::milliseconds(5);
constexpr auto kReplyTimeout = std::chrono::milliseconds(300);
constexpr auto kDeviceInfoTimeout = std::chrono::milliseconds(500);
// The test takes ~3.7 s, during which the NavX3 sends nothing at all.
constexpr auto kSelfTestTimeout = std::chrono::milliseconds(5000);
// Re-wake a NavX3 that has been quiet this long (e.g. after it reset).
constexpr auto kWakeInterval = std::chrono::seconds(1);
constexpr size_t kMaxLine = 1024;

bool isKeyChar(char c)
{
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}

/** The text after "<key>:" where key starts a word, or nullptr if absent. */
const char* findField(const std::string& line, const char* key)
{
    const size_t key_len = std::strlen(key);
    for (size_t at = line.find(key); at != std::string::npos; at = line.find(key, at + 1)) {
        const size_t colon = at + key_len;
        if ((at == 0 || !isKeyChar(line[at - 1])) && colon < line.size() && line[colon] == ':') {
            const char* value = line.c_str() + colon + 1;
            while (*value == ' ' || *value == '\t') value++;
            return value;
        }
    }
    return nullptr;
}

/** Parses "[a,b,...]" into out (up to n values). Returns the count read. */
int parseIntList(const char* value, long long* out, int n)
{
    if (value == nullptr || *value != '[') return 0;
    value++;
    int count = 0;
    while (count < n) {
        char* end = nullptr;
        const long long v = std::strtoll(value, &end, 10);
        if (end == value) break;
        out[count++] = v;
        if (*end != ',') break;
        value = end + 1;
    }
    return count;
}

/** Reads exactly n values for key, scaled. */
bool readScaled(const std::string& line, const char* key, double scale, double* out, int n)
{
    long long v[4] = {0};
    if (parseIntList(findField(line, key), v, n) != n) return false;
    for (int i = 0; i < n; i++) out[i] = v[i] / scale;
    return true;
}

bool readInt(const std::string& line, const char* key, long long& out)
{
    const char* value = findField(line, key);
    if (value == nullptr) return false;
    char* end = nullptr;
    out = std::strtoll(value, &end, 10);
    return end != value;
}

/** Decodes a DATA line into `s`. Returns the NAVX3_MSG_* bits filled. */
uint32_t decodeDataLine(const std::string& line, Navx3Data& s)
{
    if (line.find("DATA:") == std::string::npos) return 0;
    uint32_t mask = 0;
    if (readScaled(line, "YAW", kAngleScale, &s.yaw_deg, 1)) mask |= NAVX3_MSG_YAW;
    double pr[2];
    if (readScaled(line, "PITCH_ROLL", kAngleScale, pr, 2)) {
        s.pitch_deg = pr[0];
        s.roll_deg = pr[1];
        mask |= NAVX3_MSG_PITCH_ROLL;
    }
    if (readScaled(line, "ANGLE", kAngleScale, &s.angle_deg, 1)) mask |= NAVX3_MSG_ANGLE;
    if (readScaled(line, "QUAT", kQuatScale, s.quat6, 4)) mask |= NAVX3_MSG_QUAT6D;
    if (readScaled(line, "QUAT9D", kQuatScale, s.quat9, 4)) mask |= NAVX3_MSG_QUAT9D;
    if (readScaled(line, "GYRO", kGyroScale, s.angular_vel_rad_per_sec, 3)) mask |= NAVX3_MSG_ANGULAR_VELOCITY;
    if (readScaled(line, "ACCEL", kAccelScale, s.linear_accel_mps2, 3)) mask |= NAVX3_MSG_LINEAR_ACCEL;
    {
        // Three or four values: x y z [norm].
        long long v[4] = {0};
        const int n = parseIntList(findField(line, "MAG"), v, 4);
        if (n >= 3) {
            for (int i = 0; i < n; i++) s.mag_micro_tesla[i] = v[i] / kMagScale;
            mask |= NAVX3_MSG_COMPASS;
        }
    }
    bool algo = readScaled(line, "BIAS", kBiasScale, s.gyro_bias, 3);
    long long flag = 0;
    if (readInt(line, "REST", flag)) {
        s.at_rest = flag != 0;
        algo = true;
    }
    if (readInt(line, "MAGDIST", flag)) {
        s.mag_undisturbed = flag == 0;  // 1 means a magnetic disturbance is detected
        algo = true;
    }
    if (algo) mask |= NAVX3_MSG_ALGO_STATES;
    if (readScaled(line, "TEMP", kTempScale, &s.temperature_c, 1)) mask |= NAVX3_MSG_TEMPERATURE;
    return mask;
}

/** "OK,<NAME>" or "ERR,<NAME>,<reason>" anywhere it starts a word; name is set. */
bool parseReply(const std::string& line, bool& ok, bool& unknown, std::string& name)
{
    for (const char* prefix : {"OK,", "ERR,"}) {
        const size_t prefix_len = std::strlen(prefix);
        for (size_t at = line.find(prefix); at != std::string::npos; at = line.find(prefix, at + 1)) {
            if (at != 0 && isKeyChar(line[at - 1])) continue;
            const std::string rest = line.substr(at + prefix_len);
            const size_t comma = rest.find(',');
            name = rest.substr(0, comma);
            if (name.empty()) continue;
            ok = prefix[0] == 'O';
            unknown = !ok && comma != std::string::npos &&
                      rest.compare(comma + 1, 15, "unknown command") == 0;
            return true;
        }
    }
    return false;
}

}  // namespace

Navx3Usb::Navx3Usb(const std::string& port, int baud) : port_(port)
{
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); fd_ = -1; return; }
    cfmakeraw(&tty);
    const speed_t spd = (baud == 9600) ? B9600 : B115200;
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); fd_ = -1; return; }
    tcflush(fd_, TCIOFLUSH);

    running_ = true;
    reader_ = std::thread(&Navx3Usb::readerLoop, this);
}

Navx3Usb::~Navx3Usb()
{
    running_ = false;
    if (reader_.joinable()) reader_.join();
    if (fd_ >= 0) ::close(fd_);
}

bool Navx3Usb::sendCommand(const std::string& cmd)
{
    if (fd_ < 0) return false;
    std::lock_guard<std::mutex> lock(write_mutex_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    const char* data = cmd.data();
    size_t left = cmd.size();
    while (left > 0) {
        const ssize_t n = ::write(fd_, data, left);
        if (n < 0) {
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) return false;
            if (std::chrono::steady_clock::now() >= deadline) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        data += n;
        left -= static_cast<size_t>(n);
    }
    // Wait for it to leave as one packet (not tcdrain(), which has no timeout).
    int queued = 0;
    while (ioctl(fd_, TIOCOUTQ, &queued) == 0 && queued > 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(kCommandSpacing);
    return true;
}

int Navx3Usb::command(const std::string& cmd, std::chrono::milliseconds timeout)
{
    if (fd_ < 0 || !running_) return NAVX3_USB_NOT_OPEN;
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    {
        // Before sending: the reply can arrive before this thread waits for it.
        std::lock_guard<std::mutex> lock(reply_mutex_);
        awaiting_ = cmd.substr(0, cmd.find(','));
        reply_ = Reply::None;
    }
    const bool sent = sendCommand(cmd);
    std::unique_lock<std::mutex> lock(reply_mutex_);
    const bool answered = sent && reply_cv_.wait_for(lock, timeout, [this] { return reply_ != Reply::None; });
    const Reply reply = reply_;
    awaiting_.clear();
    reply_ = Reply::None;
    if (!sent) return NAVX3_USB_NOT_OPEN;
    if (!answered) return NAVX3_TIMEOUT;
    switch (reply) {
        case Reply::Ok: return NAVX3_OK;
        case Reply::Unknown: return NAVX3_USB_UNKNOWN_COMMAND;
        default: return NAVX3_USB_REJECTED;
    }
}

template <typename Have>
int Navx3Usb::requestInfo(const std::string& cmd, std::chrono::milliseconds timeout, Have have)
{
    if (fd_ < 0 || !running_) return NAVX3_USB_NOT_OPEN;
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    uint64_t request;
    {
        std::lock_guard<std::mutex> lock(reply_mutex_);
        request = ++info_request_;
    }
    if (!sendCommand(cmd)) return NAVX3_USB_NOT_OPEN;
    std::unique_lock<std::mutex> lock(reply_mutex_);
    return reply_cv_.wait_for(lock, timeout, [&] { return have(request); }) ? NAVX3_OK : NAVX3_TIMEOUT;
}

int Navx3Usb::ResetYaw()
{
    return command("RESET_YAW", kReplyTimeout);
}

int Navx3Usb::SetOutputRate(uint16_t hz)
{
    return command("SET_ODR_HZ," + std::to_string(hz), kReplyTimeout);
}

int Navx3Usb::SetYawAxes(int axes)
{
    return command("YAW," + std::to_string(axes) + "D", kReplyTimeout);  // "YAW,6D" / "YAW,9D"
}

// DEVICE_INFO answers with lines including "  Firmware: 5.0.7" and
// "  Serial Number: 0023001E-31345111-37393535", then "OK,DEVICE_INFO".
int Navx3Usb::GetFirmwareVersion(int& major, int& minor, int& patch)
{
    const int status = requestInfo("DEVICE_INFO", kDeviceInfoTimeout,
                                   [this](uint64_t request) { return firmware_request_ == request; });
    if (status != NAVX3_OK) return status;
    std::lock_guard<std::mutex> lock(reply_mutex_);
    major = firmware_[0];
    minor = firmware_[1];
    patch = firmware_[2];
    return NAVX3_OK;
}

int Navx3Usb::GetSensorUuid(uint32_t uuid[3])
{
    const int status = requestInfo("DEVICE_INFO", kDeviceInfoTimeout,
                                   [this](uint64_t request) { return uuid_request_ == request; });
    if (status != NAVX3_OK) return status;
    std::lock_guard<std::mutex> lock(reply_mutex_);
    for (int i = 0; i < 3; i++) uuid[i] = uuid_[i];
    return NAVX3_OK;
}

// SELF_TEST answers "OK,SELF_TEST" then "SELF_TEST,<8 codes>", both only after the test.
int Navx3Usb::SelfTest(int32_t results[8])
{
    quiet_until_ns_ = (std::chrono::steady_clock::now() + kSelfTestTimeout).time_since_epoch().count();
    const int status = requestInfo("SELF_TEST", kSelfTestTimeout,
                                   [this](uint64_t request) { return self_test_request_ == request; });
    quiet_until_ns_ = 0;
    if (status != NAVX3_OK) return status;
    std::lock_guard<std::mutex> lock(reply_mutex_);
    for (int i = 0; i < 8; i++) results[i] = self_test_[i];
    return NAVX3_OK;
}

bool Navx3Usb::Read(Navx3Data& out, std::chrono::milliseconds maxAge)
{
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    out = sample_;
    out.valid_mask = 0;
    for (int i = 0; i < 10; i++) {
        if (stamps_[i] != std::chrono::steady_clock::time_point{} && now - stamps_[i] <= maxAge)
            out.valid_mask |= 1u << i;
    }
    return out.valid_mask != 0;
}

void Navx3Usb::readerLoop()
{
    // A freshly powered NavX3 doesn't stream until it hears from the host.
    sendCommand("HELP");
    auto last_wake = std::chrono::steady_clock::now();

    std::string line;
    char buf[512];
    while (running_) {
        pollfd pfd{fd_, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 20);
        if (pr < 0 && errno != EINTR) break;
        if (pr > 0 && (pfd.revents & (POLLHUP | POLLERR | POLLNVAL))) {
            std::fprintf(stderr, "NavX3 USB: %s disconnected\n", port_.c_str());
            break;
        }
        if (pr > 0) {
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) break;
            for (ssize_t i = 0; i < n; i++) {
                const char c = buf[i];
                if (c == '\n' || c == '\r') {
                    if (!line.empty()) onLine(line);
                    line.clear();
                } else if (line.size() < kMaxLine) {
                    line.push_back(c);
                }
            }
        }

        const auto now = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point last_data;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_data = last_data_;
        }
        if (now - last_data > kWakeInterval && now - last_wake > kWakeInterval &&
            now.time_since_epoch().count() > quiet_until_ns_.load()) {
            sendCommand("HELP");
            last_wake = now;
        }
    }
    running_ = false;  // commands now fail at once instead of timing out
}

void Navx3Usb::onLine(const std::string& line)
{
    {
        Navx3Data s;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            s = sample_;
        }
        const uint32_t mask = decodeDataLine(line, s);
        if (mask != 0) {
            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(data_mutex_);
            sample_ = s;
            for (int i = 0; i < 10; i++) {
                if (mask & (1u << i)) stamps_[i] = now;
            }
            last_data_ = now;
        }
    }

    std::lock_guard<std::mutex> lock(reply_mutex_);
    bool notify = false;
    if (const char* fw = std::strstr(line.c_str(), "Firmware:")) {
        int major = 0, minor = 0, patch = 0;
        if (std::sscanf(fw + 9, " %d.%d.%d", &major, &minor, &patch) == 3) {
            firmware_[0] = major;
            firmware_[1] = minor;
            firmware_[2] = patch;
            firmware_request_ = info_request_;
            notify = true;
        }
    } else if (const char* sn = std::strstr(line.c_str(), "Serial Number:")) {
        unsigned w0 = 0, w1 = 0, w2 = 0;
        if (std::sscanf(sn + 14, " %x-%x-%x", &w0, &w1, &w2) == 3) {
            uuid_[0] = w0;
            uuid_[1] = w1;
            uuid_[2] = w2;
            uuid_request_ = info_request_;
            notify = true;
        }
    } else if (const char* st = std::strstr(line.c_str(), "SELF_TEST,")) {
        // "OK,SELF_TEST" and the HELP text contain the key too, but no codes follow it there.
        int v[8];
        if (std::sscanf(st + 10, "%d,%d,%d,%d,%d,%d,%d,%d",
                        &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]) == 8) {
            for (int i = 0; i < 8; i++) self_test_[i] = v[i];
            self_test_request_ = info_request_;
            notify = true;
        }
    }

    bool ok = false, unknown = false;
    std::string name;
    if (!awaiting_.empty() && reply_ == Reply::None && parseReply(line, ok, unknown, name) &&
        name == awaiting_) {
        reply_ = ok ? Reply::Ok : (unknown ? Reply::Unknown : Reply::Error);
        notify = true;
    }
    if (notify) reply_cv_.notify_all();
}

}  // namespace studica_driver
