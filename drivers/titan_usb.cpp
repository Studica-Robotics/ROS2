#include "titan_usb.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

bool startsWith(const std::string& s, const char* prefix)
{
    return s.rfind(prefix, 0) == 0;
}

// Find a line beginning with `prefix`; return the text after it, or "" if none.
std::string valueAfter(const std::vector<std::string>& lines, const char* prefix)
{
    const size_t plen = std::strlen(prefix);
    for (const auto& line : lines) {
        if (line.rfind(prefix, 0) == 0) {
            return line.substr(plen);
        }
    }
    return std::string();
}

}  // namespace

TitanUsb::TitanUsb(const std::string& port, uint16_t motorFreq, int baud)
    : port_(port), motor_freq_(motorFreq)
{
    if (!openPort(baud)) {
        return;
    }

    // Start draining the port before we send anything so we never miss replies.
    running_ = true;
    reader_ = std::thread(&TitanUsb::readerLoop, this);

    // Apply the PWM frequency to all four channels (matches the CAN driver).
    if (motor_freq_ > 0 && motor_freq_ <= 20000) {
        for (int ch = 0; ch < kNumMotors; ++ch) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "SetupMotors %d %u", ch, static_cast<unsigned>(motor_freq_));
            SendCommand(buf);
        }
    }
}

TitanUsb::~TitanUsb()
{
    // On clean teardown, disarm the timeout and actively stop the motors
    // (USBOverride 0 alone does not zero the outputs).
    if (fd_ >= 0) {
        SendCommand("USBTimeout 0");
        stopAllMotors();
        SendCommand("USBOverride 0");
    }

    running_ = false;
    if (reader_.joinable()) {
        reader_.join();
    }
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool TitanUsb::openPort(int baud)
{
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close(fd_);
        fd_ = -1;
        return false;
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
        return false;
    }
    return true;
}

bool TitanUsb::SendCommand(const std::string& cmd)
{
    if (fd_ < 0) {
        return false;
    }
    std::string line = cmd;
    if (line.empty() || line.back() != '\n') {
        line += "\r\n";
    }

    std::lock_guard<std::mutex> lock(tx_mutex_);  // keep each command line atomic on the wire
    const char* data = line.c_str();
    size_t left = line.size();
    while (left > 0) {
        const ssize_t n = write(fd_, data, left);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return false;
        }
        data += n;
        left -= static_cast<size_t>(n);
    }
    return true;
}

void TitanUsb::readerLoop()
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

        // Assemble bytes into complete '\n'-terminated lines (strip trailing '\r').
        for (ssize_t i = 0; i < n; ++i) {
            const char c = static_cast<char>(chunk[i]);
            if (c == '\n') {
                if (!rx_partial_.empty() && rx_partial_.back() == '\r') {
                    rx_partial_.pop_back();
                }
                std::lock_guard<std::mutex> lock(rx_mutex_);
                rx_lines_.push_back(rx_partial_);
                if (rx_lines_.size() > 256) {
                    rx_lines_.pop_front();  // bound memory; old replies are discardable
                }
                rx_partial_.clear();
            } else {
                rx_partial_.push_back(c);
                if (rx_partial_.size() > 512) {
                    rx_partial_.clear();  // guard against a never-terminated line
                }
            }
        }
    }
}

std::vector<std::string> TitanUsb::transact(
    const std::string& cmd, int timeout_ms,
    const std::function<bool(const std::vector<std::string>&)>& complete)
{
    std::lock_guard<std::mutex> q(query_mutex_);  // one transaction at a time

    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        rx_lines_.clear();  // drop anything buffered so the reply we collect is ours
    }

    std::vector<std::string> out;
    if (!SendCommand(cmd)) {
        return out;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        bool saw_eor = false;
        {
            std::lock_guard<std::mutex> lock(rx_mutex_);
            while (!rx_lines_.empty()) {
                std::string line = std::move(rx_lines_.front());
                rx_lines_.pop_front();
                if (startsWith(line, "EOR=")) {
                    saw_eor = true;     // explicit end-of-response marker
                    break;
                }
                out.push_back(std::move(line));
            }
        }
        if (saw_eor || (complete && complete(out))) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return out;
}

// --- enable / open-loop ------------------------------------------------------

void TitanUsb::Enable(bool enable)
{
    if (enable) {
        // USBOverride 1 lets the device accept motor commands over USB and
        // clears the disabled state.
        SendCommand("USBOverride 1");
        SendCommand("EnableRobot");
        // Arm the firmware command timeout (stops motors if the host goes
        // silent); the periodic SetMotorSpeed resend keeps it fed.
        char buf[32];
        std::snprintf(buf, sizeof(buf), "USBTimeout %d", kUsbTimeoutMs);
        SendCommand(buf);
        return;
    }

    // Disarm before stopping so the clean shutdown is not seen as a fault.
    SendCommand("USBTimeout 0");
    stopAllMotors();
    SendCommand("DisableRobot");
    SendCommand("USBOverride 0");
}

void TitanUsb::stopAllMotors()
{
    for (uint8_t m = 0; m < kNumMotors; ++m) {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "SetMotorSpeed %u 0 1 0", m);
        SendCommand(buf);
    }
}

void TitanUsb::SetSpeed(uint8_t motor, double speedCfg)
{
    if (motor >= kNumMotors) {
        return;
    }
    if (invert_motor_[motor]) {
        speedCfg = -speedCfg;
    }
    if (speedCfg > 1.0) speedCfg = 1.0;
    if (speedCfg < -1.0) speedCfg = -1.0;

    int duty = static_cast<int>(std::fabs(speedCfg) * 100.0);
    if (duty > 100) duty = 100;

    int inA, inB;
    if (speedCfg == 0.0) {
        inA = 1; inB = 1;        // brake
    } else if (speedCfg > 0.0) {
        inA = 1; inB = 0;        // forward
    } else {
        inA = 0; inB = 1;        // reverse
    }

    // Resend unconditionally (like the CAN driver); this also feeds the
    // firmware command timeout.
    char buf[48];
    std::snprintf(buf, sizeof(buf), "SetMotorSpeed %u %d %d %d", motor, duty, inA, inB);
    SendCommand(buf);
}

void TitanUsb::SetSpeedAll(double duty)
{
    if (duty > 1.0) duty = 1.0;
    if (duty < 0.0) duty = 0.0;  
    for (uint8_t m = 0; m < kNumMotors; ++m) {
        SetSpeed(m, duty);
    }
}

void TitanUsb::DisableMotor(uint8_t motor)
{
    if (motor >= kNumMotors) {
        return;
    }
    // No dedicated USB command; the CAN DISABLE_MOTOR just drives the channel to
    // a coast stop (speed 0, both inputs low), so do the same here.
    char buf[48];
    std::snprintf(buf, sizeof(buf), "SetMotorSpeed %u 0 0 0", motor);
    SendCommand(buf);
}

void TitanUsb::SetMotorStopMode(uint8_t mode)
{
    // SetStopMode <0|1>: 0=coast, 1=brake (same as CAN SET_MOTOR_STOP_MODE).
    char buf[32];
    std::snprintf(buf, sizeof(buf), "SetStopMode %u", static_cast<unsigned>(mode));
    SendCommand(buf);
}

// --- closed-loop -------------------------------------------------------------

void TitanUsb::SetTargetVelocity(uint8_t motor, float velocityRpm)
{
    if (motor >= kNumMotors) {
        return;
    }
    char buf[48];
    std::snprintf(buf, sizeof(buf), "SetV %u %.2f", motor, static_cast<double>(velocityRpm));
    SendCommand(buf);
}

void TitanUsb::SetTargetDistance(uint8_t motor, int32_t distanceCounts)
{
    if (motor >= kNumMotors) {
        return;
    }
    char buf[48];
    std::snprintf(buf, sizeof(buf), "Setp %u %ld", motor, static_cast<long>(distanceCounts));
    SendCommand(buf);
}

void TitanUsb::SetTargetAngle(uint8_t motor, double angleDeg)
{
    if (motor >= kNumMotors) {
        return;
    }
    // Setp target is interpreted as an angle when the channel runs a Cypher
    // (absolute) encoder, matching the CAN SET_TARGET_ANGLE path.
    char buf[48];
    std::snprintf(buf, sizeof(buf), "Setp %u %.2f", motor, angleDeg);
    SendCommand(buf);
}

void TitanUsb::SetPositionHold(uint8_t motor, bool hold)
{
    if (motor >= kNumMotors) {
        return;
    }
    char buf[48];
    std::snprintf(buf, sizeof(buf), "SetPositionHold %u %d", motor, hold ? 1 : 0);
    SendCommand(buf);
}

// --- pid ---------------------------------------------------------------------

void TitanUsb::SetPIDType(uint8_t type)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "PIDType %u", type);
    SendCommand(buf);
}

void TitanUsb::SetMotorPIDType(uint8_t motor, uint8_t type)
{
    if (motor >= kNumMotors) {
        return;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "PIDType_%u %u", motor, type);
    SendCommand(buf);
}

void TitanUsb::SetSensitivity(uint8_t motor, uint8_t sensitivity)
{
    if (motor >= kNumMotors) {
        return;
    }
    // EEPROM-backed sensitivity (0..10); persists across reboots, unlike CAN.
    char buf[48];
    std::snprintf(buf, sizeof(buf), "Wrt_CURVE_SENSITIVITY_%u %u", motor, sensitivity);
    SendCommand(buf);
}

void TitanUsb::AutotuneAll()
{
    SendCommand("Autotune all");
}

void TitanUsb::AutotuneMotor(uint8_t motor)
{
    if (motor >= kNumMotors) {
        return;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "Autotune %u", motor);
    SendCommand(buf);
}

// --- encoder configuration ---------------------------------------------------

void TitanUsb::SetupEncoder(uint8_t encoder)
{
    // Distance/tick is tracked host-side (ConfigureEncoder) and reset is a
    // separate call (ResetEncoder), so nothing to send here.
    (void)encoder;
}

void TitanUsb::ConfigureEncoder(uint8_t motor, double cfg)
{
    if (motor < kNumMotors) {
        dist_per_tick_[motor] = cfg;  // distance is computed host-side from counts
    }
}

void TitanUsb::SetEncoderResolution(uint8_t channel, uint16_t cpr)
{
    if (channel >= kNumMotors) {
        return;
    }
    // EEPROM-backed resolution (PPR); persists, unlike the CAN runtime value.
    char buf[48];
    std::snprintf(buf, sizeof(buf), "Wrt_ENCODER_RESOLUTION_%u %u", channel, cpr);
    SendCommand(buf);
}

void TitanUsb::ResetEncoder(uint8_t motor)
{
    if (motor >= kNumMotors) {
        return;
    }
    // Per-channel reset; matches CAN RESET_ENCODER (firmware "ResetEncoder <ch>").
    char buf[32];
    std::snprintf(buf, sizeof(buf), "ResetEncoder %u", motor);
    SendCommand(buf);
}

// --- current limiting --------------------------------------------------------

void TitanUsb::SetCurrentLimit(uint8_t channel, float limitAmps)
{
    if (channel >= kNumMotors) {
        return;
    }
    if (limitAmps < 0.0f) {
        limitAmps = 0.0f;
    }
    // Write_CURRENT_LIMIT_x takes amps as a decimal string (EEPROM-backed).
    char buf[48];
    std::snprintf(buf, sizeof(buf), "Write_CURRENT_LIMIT_%u %.3f", channel, static_cast<double>(limitAmps));
    SendCommand(buf);
}

void TitanUsb::SetCurrentLimitMode(uint8_t channel, uint8_t mode)
{
    if (channel >= kNumMotors) {
        return;
    }
    char buf[48];
    std::snprintf(buf, sizeof(buf), "SetCurrentLimitMode %u %d", channel, mode != 0 ? 1 : 0);
    SendCommand(buf);
}

// --- inversion (host-side, identical semantics to the CAN driver) ------------

void TitanUsb::InvertMotor(uint8_t motor)
{
    InvertMotorDirection(motor);
    InvertMotorRPM(motor);
    InvertEncoderDirection(motor);
}

void TitanUsb::InvertMotorDirection(uint8_t motor)
{
    if (motor < kNumMotors) {
        invert_motor_[motor] = true;
    }
}

void TitanUsb::InvertMotorRPM(uint8_t motor)
{
    if (motor < kNumMotors) {
        invert_rpm_[motor] = true;
    }
}

void TitanUsb::InvertEncoderDirection(uint8_t motor)
{
    if (motor < kNumMotors) {
        invert_encoder_[motor] = true;
    }
}

// --- telemetry polling -------------------------------------------------------

void TitanUsb::refreshCountersIfStale()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (counters_time_.time_since_epoch().count() != 0 &&
        now - counters_time_ < std::chrono::milliseconds(kCacheValidMs)) {
        return;
    }

    // ReadCounter prints "Encoder N = <count> ..." for 0..3 then "MN RPM = <f>".
    // It has no EOR line, so complete once we have seen the last RPM line.
    auto lines = transact("ReadCounter", 80, [](const std::vector<std::string>& l) {
        for (const auto& s : l) {
            if (startsWith(s, "M3 RPM")) {
                return true;
            }
        }
        return false;
    });

    for (const auto& line : lines) {
        int idx = 0;
        long count = 0;
        double rpm = 0.0;
        if (std::sscanf(line.c_str(), "Encoder %d = %ld", &idx, &count) == 2) {
            if (idx >= 0 && idx < kNumMotors) {
                enc_count_raw_[idx] = static_cast<int32_t>(count);
            }
        } else if (std::sscanf(line.c_str(), "M%d RPM = %lf", &idx, &rpm) == 2) {
            if (idx >= 0 && idx < kNumMotors) {
                rpm_raw_[idx] = rpm;
            }
        }
    }
    counters_time_ = now;
}

void TitanUsb::refreshCypherIfStale()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (cypher_time_.time_since_epoch().count() != 0 &&
        now - cypher_time_ < std::chrono::milliseconds(kCacheValidMs)) {
        return;
    }

    // ReadCypher prints "MN Angle: <deg>" for 0..3 (no EOR line).
    auto lines = transact("ReadCypher", 80, [](const std::vector<std::string>& l) {
        for (const auto& s : l) {
            if (startsWith(s, "M3 Angle")) {
                return true;
            }
        }
        return false;
    });

    for (const auto& line : lines) {
        int idx = 0;
        double deg = 0.0;
        if (std::sscanf(line.c_str(), "M%d Angle: %lf", &idx, &deg) == 2) {
            if (idx >= 0 && idx < kNumMotors) {
                cypher_deg_[idx] = deg;
            }
        }
    }
    cypher_time_ = now;
}

void TitanUsb::refreshLimitsIfStale()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (limits_time_.time_since_epoch().count() != 0 &&
        now - limits_time_ < std::chrono::milliseconds(kCacheValidMs)) {
        return;
    }

    // ReadLimitSwitches prints "MN HIGH=<0|1> LOW=<0|1>" for 0..3, then EOR.
    auto lines = transact("ReadLimitSwitches", 80, nullptr);
    for (const auto& line : lines) {
        int idx = 0, hi = 0, lo = 0;
        if (std::sscanf(line.c_str(), "M%d HIGH=%d LOW=%d", &idx, &hi, &lo) == 3) {
            if (idx >= 0 && idx < kNumMotors) {
                ls_high_[idx] = (hi != 0);
                ls_low_[idx] = (lo != 0);
            }
        }
    }
    limits_time_ = now;
}

// --- read back ---------------------------------------------------------------

float TitanUsb::GetRPM(uint8_t motor)
{
    if (motor >= kNumMotors) {
        return 0.0f;
    }
    refreshCountersIfStale();
    std::lock_guard<std::mutex> lock(cache_mutex_);
    double v = rpm_raw_[motor];
    return static_cast<float>(invert_rpm_[motor] ? -v : v);
}

int32_t TitanUsb::GetEncoderCount(uint8_t motor)
{
    if (motor >= kNumMotors) {
        return 0;
    }
    refreshCountersIfStale();
    std::lock_guard<std::mutex> lock(cache_mutex_);
    int32_t c = enc_count_raw_[motor];
    return invert_encoder_[motor] ? -c : c;
}

double TitanUsb::GetEncoderDistance(uint8_t motor)
{
    if (motor >= kNumMotors) {
        return 0.0;
    }
    return static_cast<double>(GetEncoderCount(motor)) * dist_per_tick_[motor];
}

bool TitanUsb::TryGetTargetRPMFromAll(float targetRpm[4])
{
    if (targetRpm == nullptr) {
        return false;
    }
    for (int i = 0; i < kNumMotors; ++i) {
        targetRpm[i] = 0.0f;
    }
    // GetTargetRPM prints "MN TargetRPM=<f> (PIDType=...)" for 0..3, then EOR.
    auto lines = transact("GetTargetRPM", 120, nullptr);
    bool any = false;
    for (const auto& line : lines) {
        int idx = 0;
        double v = 0.0;
        if (std::sscanf(line.c_str(), "M%d TargetRPM=%lf", &idx, &v) == 2) {
            if (idx >= 0 && idx < kNumMotors) {
                targetRpm[idx] = static_cast<float>(v);
                any = true;
            }
        }
    }
    return any;
}

double TitanUsb::GetCypherAngle(uint8_t port)
{
    if (port >= kNumMotors) {
        return 0.0;
    }
    refreshCypherIfStale();
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return cypher_deg_[port];
}

bool TitanUsb::GetLimitSwitch(uint8_t motor, uint8_t direction)
{
    if (motor >= kNumMotors) {
        return false;
    }
    refreshLimitsIfStale();
    std::lock_guard<std::mutex> lock(cache_mutex_);
    // direction 0 = forward = HIGH limit, 1 = reverse = LOW limit (matches CAN).
    return direction == 0 ? ls_high_[motor] : ls_low_[motor];
}

float TitanUsb::GetControllerTemp()
{
    // ReadTemp -> "MCU_TEMP=<int> <dec>"; temp = int + dec/100 (matches CAN).
    auto lines = transact("ReadTemp", 120, nullptr);
    const std::string v = valueAfter(lines, "MCU_TEMP=");
    int t_int = 0, t_dec = 0;
    if (v.empty() || std::sscanf(v.c_str(), "%d %d", &t_int, &t_dec) < 1) {
        return 0.0f;
    }
    return static_cast<float>(t_int) + static_cast<float>(t_dec) / 100.0f;
}

std::string TitanUsb::GetFirmwareVersion()
{
    auto lines = transact("ReadFirmwareVersion", 120, [](const std::vector<std::string>& l) {
        for (const auto& s : l) {
            if (startsWith(s, "FIRMWARE_VERSION=")) {
                return true;
            }
        }
        return false;
    });
    const std::string v = valueAfter(lines, "FIRMWARE_VERSION=");
    if (v.empty()) {
        return "Firmware Version: (read failed)";
    }
    return "Firmware Version: [" + v + "]";  // v is "maj.min.build"
}

std::string TitanUsb::GetHardwareVersion()
{
    auto type_lines = transact("ReadHardwareType", 120, [](const std::vector<std::string>& l) {
        for (const auto& s : l) {
            if (startsWith(s, "HARDWARE_TYPE=")) {
                return true;
            }
        }
        return false;
    });
    auto rev_lines = transact("ReadHardwareVersion", 120, [](const std::vector<std::string>& l) {
        for (const auto& s : l) {
            if (startsWith(s, "HARDWARE_VERSION=")) {
                return true;
            }
        }
        return false;
    });

    const std::string type_str = valueAfter(type_lines, "HARDWARE_TYPE=");
    const std::string rev_str = valueAfter(rev_lines, "HARDWARE_VERSION=");
    if (type_str.empty() || rev_str.empty()) {
        return "Hardware: (read failed)";
    }

    const int type = std::atoi(type_str.c_str());
    if (type == 1) {
        return "Hardware: Titan Quad, Version: " + rev_str;
    }
    if (type == 2) {
        return "Hardware: Titan Small, Version: " + rev_str;
    }
    return "Hardware: Type " + type_str + ", Rev " + rev_str + " (unknown type)";
}

std::string TitanUsb::GetSerialNumber()
{
    auto lines = transact("ReadUniqueID", 120, [](const std::vector<std::string>& l) {
        for (const auto& s : l) {
            if (startsWith(s, "UNIQUE_ID=")) {
                return true;
            }
        }
        return false;
    });
    const std::string v = valueAfter(lines, "UNIQUE_ID=");
    return v.empty() ? std::string("(read failed)") : v;  // "AAAAAAAA-BBBBBBBB-CCCCCCCC"
}

uint8_t TitanUsb::GetID()
{
    // No dedicated command; CAN_ID is the first line of GetInfo (ends with EOR).
    auto lines = transact("GetInfo", 250, nullptr);
    const std::string v = valueAfter(lines, "CAN_ID=");
    return v.empty() ? 0 : static_cast<uint8_t>(std::atoi(v.c_str()));
}

// --- read back with freshness (USB is always "fresh": a poll just happened) --

bool TitanUsb::GetEncoderDistanceFresh(uint8_t motor, double& distance, bool& is_fresh,
                                       uint64_t* /*out_timestamp_us*/)
{
    if (motor >= kNumMotors) {
        is_fresh = false;
        return false;
    }
    distance = GetEncoderDistance(motor);
    is_fresh = true;
    return true;
}

bool TitanUsb::GetRPMFresh(uint8_t motor, float& rpm, bool& is_fresh,
                           uint64_t* /*out_timestamp_us*/)
{
    if (motor >= kNumMotors) {
        is_fresh = false;
        return false;
    }
    rpm = GetRPM(motor);
    is_fresh = true;
    return true;
}

bool TitanUsb::GetCypherAnglesFresh(double angles[4], bool& is_fresh,
                                    uint64_t* /*out_timestamp_us*/)
{
    refreshCypherIfStale();
    std::lock_guard<std::mutex> lock(cache_mutex_);
    for (int i = 0; i < kNumMotors; ++i) {
        angles[i] = cypher_deg_[i];
    }
    is_fresh = true;
    return true;
}

bool TitanUsb::GetLimitSwitchesFresh(bool fwd[4], bool rev[4], bool& is_fresh,
                                     uint64_t* /*out_timestamp_us*/)
{
    refreshLimitsIfStale();
    std::lock_guard<std::mutex> lock(cache_mutex_);
    for (int i = 0; i < kNumMotors; ++i) {
        fwd[i] = ls_high_[i];   // HIGH limit (read_limit dir 1), matching CAN
        rev[i] = ls_low_[i];    // LOW  limit (read_limit dir 0)
    }
    is_fresh = true;
    return true;
}

}  // namespace studica_driver
