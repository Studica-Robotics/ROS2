/**
 * Colore CAN exercise program — runs every host API that maps to firmware CAN2 commands.
 *
 * Prerequisite: implement Colore::Write / Read / ReadCached / PullCANData in Colore.cpp
 * using your VMXPi stack (same as Parsec / Titan), then link this file into a small executable.
 *
 * Build (example, after VMX paths are set):
 *   g++ -std=c++17 -I/path/to/vmx Colore.cpp Colore_can_test.cpp ... -o colore_can_test
 */

#include "Colore.h"
#include "VMXPi.h"
#include "VMXCAN.h"
#include "VMXErrors.h"

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#if defined(__linux__) || defined(__unix__)
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <dirent.h>
#include <cerrno>
#include <cstring>
#endif

namespace {

unsigned ReadEnvUInt(const char* name, unsigned defv)
{
    const char* s = std::getenv(name);
    if (!s || !*s) return defv;
    return static_cast<unsigned>(std::strtoul(s, nullptr, 0));
}

void ScanCanOnBoot(const std::shared_ptr<VMXPi>& vmx)
{
    if (!vmx || !vmx->IsOpen()) {
        std::fprintf(stderr, "ScanCanOnBoot: VMXPi not open.\n");
        return;
    }

    const unsigned ch = ReadEnvUInt("COLORE_CAN_CHANNEL", ReadEnvUInt("PARSEC_CAN_CHANNEL", 0u));
    std::printf("\n=== CAN boot scan (channel=%u) ===\n", ch);

    // 1) Bus status (works even if RX stream creation fails)
    VMXCANBusStatus st;
    VMXErrorCode ec = static_cast<VMXErrorCode>(0);
    const bool okSt = vmx->can.GetCANBUSStatus(st, &ec);
    std::printf("CAN GetCANBUSStatus: %s (VMXErrorCode=%d)\n", okSt ? "OK" : "FAIL", (int)ec);
    std::printf("  busWarning=%u busPassiveError=%u busOffError=%u\n",
                st.busWarning ? 1u : 0u,
                st.busPassiveError ? 1u : 0u,
                st.busOffError ? 1u : 0u);
    std::printf("  txFullCount=%u rxErrorCount=%u txErrorCount=%u hwRxOverflow=%u swRxOverflow=%u\n",
                static_cast<unsigned>(st.txFullCount),
                static_cast<unsigned>(st.receiveErrorCount),
                static_cast<unsigned>(st.transmitErrorCount),
                st.hwRxOverflow ? 1u : 0u,
                st.swRxOverflow ? 1u : 0u);

    // 2) Try to open a catch-all receive stream and print what arrives briefly.
    // Note: this consumes HW filter resources temporarily; we close it afterwards.
    std::printf("\nAttempt catch-all RX stream for ~200ms...\n");
    (void)vmx->can.SetMode(VMXCAN::VMXCAN_NORMAL, nullptr);
    vmx->time.DelayMilliseconds(20);

    (void)vmx->can.FlushRxFIFO(nullptr);
    (void)vmx->can.FlushTxFIFO(nullptr);
    (void)vmx->can.Reset(nullptr);

    VMXCANReceiveStreamHandle h = 0;
    VMXErrorCode ec2 = static_cast<VMXErrorCode>(0);
    const bool opened = vmx->can.OpenReceiveStream(h, 0u, 0u, 64u, &ec2);
    std::printf("OpenReceiveStream(catch-all): %s (VMXErrorCode=%d, handle=%u)\n",
                opened ? "OK" : "FAIL", (int)ec2, static_cast<unsigned>(h));
    if (!opened)
        return;

    (void)vmx->can.EnableReceiveStreamBlackboard(h, true, nullptr);

    uint32_t totalRead = 0u;
    VMXCANTimestampedMessage msgs[64];
    for (int i = 0; i < 20; ++i) { // 20 * 10ms = 200ms
        (void)vmx->can.RetrieveAllCANData(static_cast<uint32_t>(ch), nullptr);
        uint32_t nread = 0u;
        (void)vmx->can.ReadReceiveStream(h, msgs, 64u, nread, nullptr);
        for (uint32_t k = 0u; k < nread; ++k) {
            const uint32_t id = msgs[k].messageID & VMXCAN_29BIT_MESSAGE_ID_MASK;
            std::printf("  RX id=0x%08X dlc=%u data0=%02X\n",
                        static_cast<unsigned>(id),
                        static_cast<unsigned>(msgs[k].dataSize),
                        msgs[k].data[0]);
        }
        totalRead += nread;
        vmx->time.DelayMilliseconds(10);
    }
    std::printf("Catch-all RX total frames: %u\n", static_cast<unsigned>(totalRead));

    (void)vmx->can.CloseReceiveStream(h, nullptr);
}

#if defined(__linux__) || defined(__unix__)
std::string FindColoreUsbTTY()
{
    const char* env = std::getenv("COLORE_USB_TTY");
    if (env && *env) return std::string(env);

    auto matchesPrefix = [](const char* name, const char* prefix) -> bool {
        if (!name || !prefix) return false;
        const size_t nlen = std::strlen(name);
        const size_t plen = std::strlen(prefix);
        if (nlen < plen) return false;
        return std::strncmp(name, prefix, plen) == 0;
    };

    auto pickFirstMatching = [matchesPrefix](const char* prefix) -> std::string {
        DIR* d = opendir("/dev");
        if (!d) return {};
        std::string best;
        while (true) {
            errno = 0;
            dirent* ent = readdir(d);
            if (!ent) break;
            if (!matchesPrefix(ent->d_name, prefix)) continue;
            std::string full = std::string("/dev/") + ent->d_name;
            if (access(full.c_str(), F_OK) != 0) continue;
            // Choose smallest lexicographically (e.g. ttyACM0 before ttyACM1)
            if (best.empty() || full < best) best = full;
        }
        closedir(d);
        return best;
    };

    // Diagnostics: list what we can see.
    std::printf("USB CDC TTY lookup: COLORE_USB_TTY not set; scanning /dev for ttyACM*/ttyUSB*...\n");

    auto list_once = [&]() {
        DIR* d = opendir("/dev");
        if (d) {
            std::printf("Candidates present:\n");
            while (true) {
                errno = 0;
                dirent* ent = readdir(d);
                if (!ent) break;
                if (matchesPrefix(ent->d_name, "ttyACM") || matchesPrefix(ent->d_name, "ttyUSB")) {
                    std::printf("  /dev/%s\n", ent->d_name);
                }
            }
            closedir(d);
        }

        DIR* sd = opendir("/dev/serial/by-id");
        if (sd) {
            std::printf("Also /dev/serial/by-id entries:\n");
            while (true) {
                errno = 0;
                dirent* ent = readdir(sd);
                if (!ent) break;
                if (!ent->d_name || std::string(ent->d_name) == "." || std::string(ent->d_name) == "..") continue;
                std::string full = std::string("/dev/serial/by-id/") + ent->d_name;
                char target[512];
                ssize_t n = readlink(full.c_str(), target, sizeof(target) - 1);
                if (n > 0) {
                    target[n] = '\0';
                    std::printf("  %s -> %s\n", ent->d_name, target);
                } else {
                    std::printf("  %s -> (unreadable)\n", ent->d_name);
                }
            }
            closedir(sd);
        }
    };

    auto scan_once_and_print = [&]() -> std::string {
        std::string a = pickFirstMatching("ttyACM");
        std::string u = pickFirstMatching("ttyUSB");
        if (!a.empty()) {
            std::printf("Found USB CDC ttyACM candidate: %s\n", a.c_str());
            return a;
        }
        if (!u.empty()) {
            std::printf("Found USB CDC ttyUSB candidate: %s\n", u.c_str());
            return u;
        }

        return {};
    };

    // Wait a bit for enumeration (USB CDC sometimes appears after a moment).
    const int kWaitMs = 15000;
    const int kStepMs = 500;

    list_once();
    for (int waited = 0; waited <= kWaitMs; waited += kStepMs) {
        std::string dev = scan_once_and_print();
        if (!dev.empty()) return dev;
        usleep(static_cast<useconds_t>(kStepMs) * 1000u);
    }

    std::printf("USB CDC TTY still not found after waiting %dms.\n", kWaitMs);
    list_once();
    return {};
}

int OpenUsbTTY(const std::string& dev)
{
    int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = (B115200 | CS8 | CLOCAL | CREAD);
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    (void)tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

bool UsbSendLine(int fd, const std::string& line)
{
    if (fd < 0) return false;
    std::string s = line;
    // firmware trims CR/LF, so ensure CRLF
    if (s.empty() || s[s.size() - 1] != '\n')
        s += "\r\n";

    const char* p = s.c_str();
    size_t left = s.size();
    while (left > 0) {
        const ssize_t n = write(fd, p, left);
        if (n <= 0) return false;
        p += static_cast<size_t>(n);
        left -= static_cast<size_t>(n);
    }
    return true;
}

void StreamXYZForSeconds(int fd, int seconds)
{
    if (fd < 0 || seconds <= 0) return;
    std::printf("Reading XYZ for %d seconds...\n", seconds);

    std::string line;
    char buf[256];
    auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);

    while (std::chrono::steady_clock::now() < t_end) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200 * 1000; // 200ms

        const int r = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (r <= 0) continue;

        const ssize_t n = read(fd, buf, sizeof(buf));
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            const char c = buf[i];
            line.push_back(c);
            if (c == '\n') {
                if (line.find("XYZ:") != std::string::npos) {
                    std::printf("%s", line.c_str());
                    std::fflush(stdout);
                }
                line.clear();
            }
        }
    }
}
#endif

void print_ack(const char* title, bool ok, uint32_t v)
{
    std::printf("%-40s %s  value=0x%08" PRIX32 "\n", title, ok ? "OK" : "FAIL", v);
}

/** Optional: startup handshake (firmware CAN_STARTUP). Classic 1-byte; FW stays CAN 2.0 only. */
bool send_startup_classic(studica_driver::Colore& colore)
{
    uint8_t one = 0;
    /* COLORE_* are macros: do not prefix with studica_driver:: */
    return colore.Write(COLORE_CAN_STARTUP, &one, 1u, 0);
}

} // namespace

/**
 * @param canID  Must match firmware device (usually 0 until multi-node IDs are on the wire).
 * @param vmx    Live VMXPi instance (CAN channel via COLORE_CAN_CHANNEL env if you support it).
 */
int RunAllColoreCanTests(uint8_t canID, std::shared_ptr<VMXPi> vmx)
{
    using namespace studica_driver;

    if (!vmx || !vmx->IsOpen()) {
        std::fprintf(stderr, "VMXPi not open; CAN will not work.\n");
        return 1;
    }

    /* Keep a copy so we can open an extra RX stream even after we move `vmx` into Colore. */
    auto vmx_rx = vmx;

    Colore colore(canID, std::move(vmx));
    if (!colore.IsCanReceiveStreamOpen()) {
        std::fprintf(stderr, "Warning: Colore ACK RX stream not open; config ACKs may fail.\n");
    }

    const unsigned ch = ReadEnvUInt("COLORE_CAN_CHANNEL", ReadEnvUInt("PARSEC_CAN_CHANNEL", 0u));
    const uint32_t teleId = COLORE_CAN_TELEM_XYZ & VMXCAN_29BIT_MESSAGE_ID_MASK;

    std::printf("Telemetry: CAN ID 0x%08X (DLC=8)\n", static_cast<unsigned>(teleId));

    /* Configure device over CAN (start non-smart streaming and switch to XYZ format). */
    uint32_t ack = 0u;
    bool okFmt = colore.SetColorFormat(Colore::ColorFormat::XYZ, &ack);
    std::printf("SetColorFormat(XYZ): %s ack=0x%08" PRIX32 "\n", okFmt ? "OK" : "FAIL", ack);

    bool okMode = colore.SetMeasureModeOff(&ack);
    std::printf("SetMeasureModeOff:    %s ack=0x%08" PRIX32 "\n", okMode ? "OK" : "FAIL", ack);

    vmx_rx->time.DelayMilliseconds(200);

    /* Open RX stream for telemetry (exact mask). */
    VMXCANReceiveStreamHandle h = 0;
    VMXErrorCode ec = static_cast<VMXErrorCode>(0);
    (void)vmx_rx->can.SetMode(VMXCAN::VMXCAN_CONFIG, nullptr);
    const bool opened = vmx_rx->can.OpenReceiveStream(h, teleId, VMXCAN_29BIT_MESSAGE_ID_MASK, 64u, &ec);
    if (!opened) {
        std::fprintf(stderr, "OpenReceiveStream telemetry failed (VMXErrorCode=%d).\n", (int)ec);
        return 1;
    }
    (void)vmx_rx->can.EnableReceiveStreamBlackboard(h, true, nullptr);
    (void)vmx_rx->can.SetMode(VMXCAN::VMXCAN_NORMAL, nullptr);
    vmx_rx->time.DelayMilliseconds(20);

    /* Read for 10 seconds. */
    std::printf("Reading XYZ telemetry for 10 seconds...\n");
    const float scale = 10000.0f;
    auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(10);

    while (std::chrono::steady_clock::now() < t_end) {
        (void)vmx_rx->can.RetrieveAllCANData(static_cast<uint32_t>(ch), nullptr);

        VMXCANTimestampedMessage msgs[32];
        uint32_t read = 0u;
        (void)vmx_rx->can.ReadReceiveStream(h, msgs, 32u, read, nullptr);
        for (uint32_t i = 0u; i < read; ++i) {
            if (msgs[i].dataSize < 8u) continue;
            const uint16_t seq = static_cast<uint16_t>(msgs[i].data[0])
                                 | (static_cast<uint16_t>(msgs[i].data[1]) << 8);
            const int16_t xi = static_cast<int16_t>(
                static_cast<uint16_t>(msgs[i].data[2]) | (static_cast<uint16_t>(msgs[i].data[3]) << 8));
            const int16_t yi = static_cast<int16_t>(
                static_cast<uint16_t>(msgs[i].data[4]) | (static_cast<uint16_t>(msgs[i].data[5]) << 8));
            const int16_t zi = static_cast<int16_t>(
                static_cast<uint16_t>(msgs[i].data[6]) | (static_cast<uint16_t>(msgs[i].data[7]) << 8));

            const float x = static_cast<float>(xi) / scale;
            const float y = static_cast<float>(yi) / scale;
            const float z = static_cast<float>(zi) / scale;
            std::printf("XYZ seq=%u  x=%.4f y=%.4f z=%.4f\n",
                        static_cast<unsigned>(seq),
                        static_cast<double>(x),
                        static_cast<double>(y),
                        static_cast<double>(z));
        }

        vmx_rx->time.DelayMilliseconds(20);
    }

    (void)vmx_rx->can.CloseReceiveStream(h, nullptr);
    std::printf("=== Done ===\n");
    return 0;
}

int main(int argc, char** argv)
{
    uint8_t canID = 0;
    if (argc >= 2) {
        canID = static_cast<uint8_t>(std::strtoul(argv[1], nullptr, 0));
    }

#if defined(__linux__) || defined(__unix__)
    if (geteuid() != 0) {
        std::fprintf(stderr, "Hint: VMX HAL on Pi often needs root for pigpio/SPI (try: sudo %s)\n",
                     argc > 0 ? argv[0] : "./Colore_example");
    }
#endif

    auto vmx = std::make_shared<VMXPi>(true, 50);
    return RunAllColoreCanTests(canID, std::move(vmx));
}
