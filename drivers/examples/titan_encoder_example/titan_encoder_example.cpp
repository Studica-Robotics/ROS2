/*
 * titan_encoder_test.cpp
 *
 * Spins motor 0 at 50% duty and polls the Titan encoder in a tight loop.
 * Each iteration reads count + VMX blackboard freshness; stale reads (no new
 * CAN frame since last poll) are skipped. On a fresh sample, prints encoder
 * count and velocity (delta_count / VMX receive timestamp dt).
 *
 * Build (from this directory, after studica_ws/drivers/ is installed):
 *   make
 *
 * Or from drivers/examples:
 *   make -C titan_encoder_test
 *
 * Run:
 *   sudo make run
 *   sudo ./titan_encoder_test [CAN_ID]
 */

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "titan.hpp"

int main(int argc, char** argv)
{
    uint8_t can_id = 20;
    if (argc >= 2)
        can_id = static_cast<uint8_t>(std::atoi(argv[1]));

    studica_driver::Titan titan(can_id, 15600, 1.0f);
    printf("%s\n", titan.GetFirmwareVersion().c_str());

    titan.Enable(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int32_t  last_enc          = 0;
    uint64_t last_timestamp_us = 0;
    double   last_output       = 0.0;

    for (int i = 0; i < 20000; i++)
    {
        titan.SetSpeed(0, 0.5);

        int32_t  enc          = 0;
        bool     is_fresh     = false;
        uint64_t timestamp_us = 0;

        titan.GetEncoderCountFresh(0, enc, is_fresh, &timestamp_us);

        if (!is_fresh)
        {
            printf("GetRPM: %d\n", titan.GetRPM(0));
            printf("Get_Last_Enc: %d\n", 0);
            printf("Titan_Raw: %" PRId32 "\n", enc);
            printf("Fresh: NO  (blackboard not updated)\n\n");

            std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1.0 * 20)));
            continue;
        }

        if (last_timestamp_us > 0)
        {
            double dt_s     = static_cast<double>(timestamp_us - last_timestamp_us) / 1e6;
            double velocity = static_cast<double>(enc - last_enc) / dt_s;

            printf("GetRPM: %d\n", titan.GetRPM(0));
            printf("Get_Last_Enc: %d\n", enc - last_enc);
            printf("Titan_Raw: %" PRId32 "\n", enc);
            printf("Fresh: YES  dt: %.1fms  Velocity: %.1f counts/sec\n\n", dt_s * 1000.0, velocity);
        }

        last_enc          = enc;
        last_timestamp_us = timestamp_us;

        std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1.0 * 20)));
    }

    titan.SetSpeed(0, 0.0);
    titan.Enable(false);
    return 0;
}
