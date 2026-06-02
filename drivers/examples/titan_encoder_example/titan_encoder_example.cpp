/*
 * titan_encoder_example.cpp
 *
 * Spins motor 0 at 50% duty and polls the Titan encoder in a tight loop.
 * Each iteration reads distance + VMX blackboard freshness; stale reads (no new
 * CAN frame since last poll) are skipped. On a fresh sample, prints distance
 * and velocity (delta_distance / VMX receive timestamp dt).
 *
 * dist_per_tick is set to 1.0 so distance == raw encoder count in ticks.
 *
 * Build (from this directory, after studica_ws/drivers/ is installed):
 *   make
 *
 * Or from drivers/examples:
 *   make -C titan_encoder_example
 *
 * Run:
 *   sudo make run
 *   sudo ./titan_encoder_example [CAN_ID]
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

    titan.ConfigureEncoder(0, 1.0);  // dist_per_tick = 1.0 so distance == raw count in ticks
    titan.Enable(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    double   last_dist         = 0.0;
    uint64_t last_timestamp_us = 0;

    for (int i = 0; i < 20000; i++)
    {
        titan.SetSpeed(0, 0.5);

        double   dist         = 0.0;
        bool     is_fresh     = false;
        uint64_t timestamp_us = 0;

        titan.GetEncoderDistanceFresh(0, dist, is_fresh, &timestamp_us);

        if (!is_fresh)
        {
            printf("GetRPM: %d\n", titan.GetRPM(0));
            printf("Fresh: NO  (blackboard not updated)\n\n");

            std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1.0 * 20)));
            continue;
        }

        if (last_timestamp_us > 0)
        {
            double dt_s     = static_cast<double>(timestamp_us - last_timestamp_us) / 1e6;
            double velocity = (dist - last_dist) / dt_s;

            printf("GetRPM: %d\n", titan.GetRPM(0));
            printf("Distance: %.4f m  Delta: %.4f m\n", dist, dist - last_dist);
            printf("Fresh: YES  dt: %.1fms  Velocity: %.3f m/s\n\n", dt_s * 1000.0, velocity);
        }

        last_dist         = dist;
        last_timestamp_us = timestamp_us;

        std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1.0 * 20)));
    }

    titan.SetSpeed(0, 0.0);
    titan.Enable(false);
    return 0;
}
