/*
 * VMX-Pi Host: Titan2 4-channel speed sequence test
 *
 * Flow:
 *   1. SetSpeed all 4 channels same duty cycle: 100% -> 50% -> 0 -> 80% -> 0 (duty cycle %)
 *   2. SetPIDType(0), SetTargetVelocity all 4 same target RPM: 100 -> 50 -> 0 -> 80 -> 0
 *   3. SetPIDType(1), same target RPM sequence again
 *   4. SetSensitivity(0..3, 10), same target RPM sequence again
 * Continuously output encoder RPM while motors are running.
 *
 * Supports 1 or 2 Titans: when two CAN IDs are passed, same commands are sent to both devices and RPM from both is printed.
 *
 * Build (Raspberry Pi, Titan2 root):
 *   g++ -o titan_speed_sequence_test VMX_HOST_TITAN_SPEED_SEQUENCE_TEST.cpp Host/titan.cpp \
 *       -I. -IHost -I/usr/local/include/vmxpi \
 *       -L/usr/local/frc/third-party/lib -lvmxpi_hal_cpp -lpthread -std=c++11
 *
 * Run: sudo ./titan_speed_sequence_test [CANID1] [CANID2]
 *      Single device: sudo ./titan_speed_sequence_test 20
 *      Two devices:   sudo ./titan_speed_sequence_test 20 21
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <unistd.h>
 #include <memory>
 #include <sys/time.h>
 #include "Host/titan.h"
 
 static inline uint64_t get_time_ms(void)
 {
     struct timeval tv;
     gettimeofday(&tv, nullptr);
     return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
 }
 
 static studica_driver::Titan* s_titans_for_atexit[2] = { nullptr, nullptr };
 static int s_num_titans_for_atexit = 0;
 static void atexit_disable_titans(void)
 {
     for (int i = 0; i < s_num_titans_for_atexit && i < 2; i++)
     {
         if (s_titans_for_atexit[i])
         {
             s_titans_for_atexit[i]->Enable(false);
             usleep(80 * 1000);
         }
     }
     usleep(400 * 1000);
 }
 
/* Hold time per speed step (seconds) */
 static const int HOLD_SEC = 5;
 /* RPM sampling interval (ms); print RPM at this interval while motor runs */
 static const int RPM_PRINT_INTERVAL_MS = 200;
 /* In Phase 1 with SetSpeed, how often to resend SET_MOTOR_SPEED. Must be < TITAN_CAN_KEEPALIVE_MS(150) to avoid device 200ms timeout disable. */
 static const int SET_SPEED_RESEND_MS = 50;

 /* SetTargetVelocity target RPM: 100, 50, 0, 80 (matches SetSpeed steps, unit rpm) */
 static const int16_t RPM_100 = 100;
 static const int16_t RPM_50  = 50;
 static const int16_t RPM_80  = 80;
 static const int16_t RPM_0   = 0;
 
/* Phase 2/3/4: periodically resend SetTargetVelocity. Must be < TITAN_CAN_KEEPALIVE_MS to avoid device 200ms timeout; print RPM every 200ms */
 static const int TARGET_VELOCITY_RESEND_MS = 50;

 /* When 2 devices: after sending 4 frames for one device, wait for VMX to put them on bus before sending to the next */
 static const int INTER_DEVICE_SEND_DELAY_MS = 20;
 /* When 2 devices: lengthen send period to 80ms so VMX has time to drain TX queue between sends; otherwise backlog from 3rd round (still < 150ms keepalive) */
 static const int SEND_PERIOD_TWO_DEVICES_MS = 80;
 
 /* Send on a strict 50ms schedule so two Titans never see ~0.5s gap caused by 8 blocking Reads. */
 static void run_for_seconds_target_velocity_and_rpm(studica_driver::Titan* titans[], int num_titans, int seconds_sec, int16_t target_rpm, const char* label)
 {
     const uint64_t total_ms = (uint64_t)seconds_sec * 1000;
     const uint64_t start_ms = get_time_ms();
     uint64_t next_send_ms = start_ms;
     uint64_t next_print_ms = start_ms;
 
     while (get_time_ms() - start_ms < total_ms)
     {
         uint64_t now_ms = get_time_ms();
         if (now_ms < next_send_ms)
         {
             uint64_t sleep_us = (next_send_ms - now_ms) * 1000;
             usleep((useconds_t)sleep_us);
             now_ms = get_time_ms();
         }
         for (int t = 0; t < num_titans; t++)
         {
             titans[t]->SetTargetVelocity(0, target_rpm);
             titans[t]->SetTargetVelocity(1, target_rpm);
             titans[t]->SetTargetVelocity(2, target_rpm);
             titans[t]->SetTargetVelocity(3, target_rpm);
             if (num_titans == 2 && t == 0)
                 usleep(INTER_DEVICE_SEND_DELAY_MS * 1000);
         }
         next_send_ms += (num_titans == 2 ? SEND_PERIOD_TWO_DEVICES_MS : TARGET_VELOCITY_RESEND_MS);
 
         if (now_ms >= next_print_ms)
         {
             next_print_ms += RPM_PRINT_INTERVAL_MS;
             usleep(8 * 1000);
             if (num_titans == 1)
             {
                 int16_t r0, r1, r2, r3;
                 bool ok0 = titans[0]->TryGetRPM(0, &r0);
                 bool ok1 = titans[0]->TryGetRPM(1, &r1);
                 bool ok2 = titans[0]->TryGetRPM(2, &r2);
                 bool ok3 = titans[0]->TryGetRPM(3, &r3);
                 printf("  [%s] Titan%u encoder RPM: M0=%d%c M1=%d%c M2=%d%c M3=%d%c\n",
                        label, (unsigned)titans[0]->GetID(), (int)r0, ok0 ? ' ' : '?', (int)r1, ok1 ? ' ' : '?', (int)r2, ok2 ? ' ' : '?', (int)r3, ok3 ? ' ' : '?');
             }
             else
             {
                 for (int t = 0; t < num_titans; t++)
                 {
                     int16_t r0;
                     bool ok0 = titans[t]->TryGetRPM(0, &r0);
                     printf("  [%s] Titan%u encoder RPM: M0=%d%c (M1..3 omitted when 2 boards to avoid blocking)\n",
                            label, (unsigned)titans[t]->GetID(), (int)r0, ok0 ? ' ' : '?');
                 }
             }
         }
     }
 }
 
 /* Stop: SetSpeedAll(0) sends 4 frames [motor, 0, 1, 0] to stop all 4. With 2 devices, short delay between them so VMX sends first device before second. */
 static void stop_all(studica_driver::Titan* titans[], int num_titans)
 {
     for (int t = 0; t < num_titans; t++)
     {
         titans[t]->SetSpeedAll(0.0);
         if (num_titans == 2 && t == 0)
             usleep(INTER_DEVICE_SEND_DELAY_MS * 1000);
     }
     usleep(150 * 1000);
 }
 
 /* Phase 1: strict 50ms send schedule; when 2 boards only read M0 to avoid ~0.5s block from 8 Reads. */
 static void run_for_seconds_set_speed_and_rpm(studica_driver::Titan* titans[], int num_titans, int seconds_sec, double duty, const char* label)
 {
     const uint64_t total_ms = (uint64_t)seconds_sec * 1000;
     const uint64_t start_ms = get_time_ms();
     uint64_t next_send_ms = start_ms;
     uint64_t next_print_ms = start_ms;
 
     while (get_time_ms() - start_ms < total_ms)
     {
         uint64_t now_ms = get_time_ms();
         if (now_ms < next_send_ms)
         {
             uint64_t sleep_us = (next_send_ms - now_ms) * 1000;
             usleep((useconds_t)sleep_us);
             now_ms = get_time_ms();
         }
         for (int t = 0; t < num_titans; t++)
         {
             if (duty == 0.0)
                 titans[t]->SetSpeedAll(0.0);
             else if (duty < 0.0)
             {
                 titans[t]->SetSpeed(0, duty);
                 usleep(3 * 1000);
                 titans[t]->SetSpeed(1, duty);
                 usleep(3 * 1000);
                 titans[t]->SetSpeed(2, duty);
                 usleep(3 * 1000);
                 titans[t]->SetSpeed(3, duty);
             }
             else
                 titans[t]->SetSpeedAll(duty);
             if (num_titans == 2 && t == 0)
                 usleep(INTER_DEVICE_SEND_DELAY_MS * 1000);
         }
         next_send_ms += (num_titans == 2 ? SEND_PERIOD_TWO_DEVICES_MS : SET_SPEED_RESEND_MS);
 
         if (now_ms >= next_print_ms)
         {
             next_print_ms += RPM_PRINT_INTERVAL_MS;
             usleep(8 * 1000);
             if (num_titans == 1)
             {
                 int16_t r0, r1, r2, r3;
                 bool ok0 = titans[0]->TryGetRPM(0, &r0);
                 bool ok1 = titans[0]->TryGetRPM(1, &r1);
                 bool ok2 = titans[0]->TryGetRPM(2, &r2);
                 bool ok3 = titans[0]->TryGetRPM(3, &r3);
                 printf("  [%s] Titan%u encoder RPM: M0=%d%c M1=%d%c M2=%d%c M3=%d%c\n",
                        label, (unsigned)titans[0]->GetID(), (int)r0, ok0 ? ' ' : '?', (int)r1, ok1 ? ' ' : '?', (int)r2, ok2 ? ' ' : '?', (int)r3, ok3 ? ' ' : '?');
             }
             else
             {
                 for (int t = 0; t < num_titans; t++)
                 {
                     int16_t r0;
                     bool ok0 = titans[t]->TryGetRPM(0, &r0);
                     printf("  [%s] Titan%u encoder RPM: M0=%d%c (M1..3 omitted when 2 boards)\n",
                            label, (unsigned)titans[t]->GetID(), (int)r0, ok0 ? ' ' : '?');
                 }
             }
         }
     }
 }
 
 int main(int argc, char** argv)
 {
     uint8_t canId1 = 42;
     uint8_t canId2 = 42;
     int num_titans = 1;
     if (argc >= 2)
         canId1 = (uint8_t)atoi(argv[1]);
     if (argc >= 3)
     {
         canId2 = (uint8_t)atoi(argv[2]);
         num_titans = 2;
     }
     else
         canId2 = canId1;
 
     printf("=== Titan2 Speed Sequence Test (4 motors) ===\n");
     printf("CAN IDs: %u", (unsigned)canId1);
     if (num_titans >= 2)
         printf(" %u (same commands to both)", (unsigned)canId2);
     printf("\n");
 
     /* Share one VMXPi so only one SPI/GPIO connection is used. Two VMXPi instances cause
      * "Failed to issue GET EVENT IOCTL", SPI timeouts, and segfault. */
     std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);
     studica_driver::Titan titan1(canId1, 2000, 1.0f, vmx);
     studica_driver::Titan titan2(canId2, 2000, 1.0f, vmx);
     studica_driver::Titan* titans[2] = { &titan1, &titan2 };
 
     s_titans_for_atexit[0] = &titan1;
     s_titans_for_atexit[1] = (num_titans >= 2) ? &titan2 : nullptr;
     s_num_titans_for_atexit = num_titans;
     atexit(atexit_disable_titans);
 
     for (int t = 0; t < num_titans; t++)
         titans[t]->Enable(true);
     sleep(1);
 
    /* Set PIDType 0 first, stop and clear target, then run SetSpeed */
    for (int t = 0; t < num_titans; t++)
         titans[t]->SetPIDType(0);
     stop_all(titans, num_titans);
     usleep(80 * 1000);
 
    /* Diagnostics: confirm device response and check if RPM frames are received */
    for (int t = 0; t < num_titans; t++)
     {
         uint8_t devId = titans[t]->GetID();
         std::string fw = titans[t]->GetFirmwareVersion();
         printf("  Titan%u: Device ID (from GET_TITAN_INFO): %u, Firmware: %s\n", (unsigned)devId, (unsigned)devId, fw.c_str());
         int ok = 0;
         for (int i = 0; i < 15; i++)
         {
             int16_t r = 0;
             if (titans[t]->TryGetRPM(0, &r))
                 ok++;
             usleep(15 * 1000);
         }
         printf("  Titan%u RPM0 frames received (before Phase 1): %d/15\n", (unsigned)devId, ok);
         if (ok == 0)
             printf("  >>> No RPM frames. Check: (1) Device CAN ID matches %u (2) Device sending sensor data every 10ms.\n", (unsigned)(t == 0 ? canId1 : canId2));
     }
 
     /* ---------- Phase 1: SetSpeed 100 -> 50 -> 0 -> 80 -> 0 ---------- */
     printf("\n--- Phase 1: SetPIDType(0), SetSpeed (duty 100%%, 50%%, 0, 80%%, 0) ---\n");
 
     printf("  SetSpeed all -> 100%% (resend every %d ms)\n", SET_SPEED_RESEND_MS);
     run_for_seconds_set_speed_and_rpm(titans, num_titans, 2, 1.0, "100%");
     printf("  SetSpeed all -> 100%% reverse (5 s)\n");
     run_for_seconds_set_speed_and_rpm(titans, num_titans, 5, -1.0, "100% rev");
     printf("  SetSpeed all -> 100%% (continue)\n");
     run_for_seconds_set_speed_and_rpm(titans, num_titans, 2, 1.0, "100%");
 
     printf("  SetSpeed all -> 50%%\n");
     run_for_seconds_set_speed_and_rpm(titans, num_titans, HOLD_SEC, 0.5, "50%");
 
     printf("  SetSpeed all -> 0 (SetSpeedAll(0) to stop)\n");
     stop_all(titans, num_titans);
     run_for_seconds_set_speed_and_rpm(titans, num_titans, 2, 0.0, "0");
 
     printf("  SetSpeed all -> 80%%\n");
     run_for_seconds_set_speed_and_rpm(titans, num_titans, HOLD_SEC, 0.8, "80%");
 
     printf("  SetSpeed all -> 0 (SetSpeedAll(0) to stop)\n");
     stop_all(titans, num_titans);
     run_for_seconds_set_speed_and_rpm(titans, num_titans, 2, 0.0, "0");
 
     usleep(150 * 1000);
 
     /* ---------- Phase 2: PIDType 0, SetTargetVelocity all 4 same target RPM 100->50->0->80->0 ---------- */
     printf("\n--- Phase 2: SetPIDType(0), SetTargetVelocity all 4 (target RPM 100->50->0->80->0) ---\n");
     for (int t = 0; t < num_titans; t++)
         titans[t]->SetPIDType(0);
 
     for (int i = 0; i < 8; i++)
     {
         for (int t = 0; t < num_titans; t++)
         {
             titans[t]->SetTargetVelocity(0, RPM_100);
             titans[t]->SetTargetVelocity(1, RPM_100);
             titans[t]->SetTargetVelocity(2, RPM_100);
             titans[t]->SetTargetVelocity(3, RPM_100);
             if (num_titans == 2 && t == 0)
                 usleep(INTER_DEVICE_SEND_DELAY_MS * 1000);
         }
         usleep(10 * 1000);
     }
 
     for (int t = 0; t < num_titans; t++)
     {
         int16_t devTarget[4] = {0, 0, 0, 0};
         if (titans[t]->GetTargetRPMFromDevice(devTarget))
             printf("  Titan%u target RPM (readback): M0=%d M1=%d M2=%d M3=%d\n",
                    (unsigned)titans[t]->GetID(), (int)devTarget[0], (int)devTarget[1], (int)devTarget[2], (int)devTarget[3]);
         else
             printf("  Titan%u target RPM readback failed.\n", (unsigned)titans[t]->GetID());
     }
     usleep(150 * 1000);
 
     printf("  SetTargetVelocity all 4 -> %d rpm (resend every %d ms)\n", (int)RPM_100, TARGET_VELOCITY_RESEND_MS);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, HOLD_SEC, RPM_100, "100rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_50);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, HOLD_SEC, RPM_50, "50rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 1, RPM_0, "0rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_80);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, HOLD_SEC, RPM_80, "80rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 1, RPM_0, "0rpm");
 
     /* ---------- Phase 3: PIDType 1 requires Autotune first, then SetTargetVelocity ---------- */
     printf("\n--- Phase 3: SetPIDType(1), AutotuneAll(), then SetTargetVelocity (same sequence) ---\n");
     for (int t = 0; t < num_titans; t++)
         titans[t]->Enable(true);
     usleep(50 * 1000);
     for (int t = 0; t < num_titans; t++)
     {
         titans[t]->SetTargetVelocity(0, 0);
         titans[t]->SetTargetVelocity(1, 0);
         titans[t]->SetTargetVelocity(2, 0);
         titans[t]->SetTargetVelocity(3, 0);
         if (num_titans == 2 && t == 0)
             usleep(INTER_DEVICE_SEND_DELAY_MS * 1000);
     }
     usleep(100 * 1000);
     for (int k = 0; k < 4; k++)
     {
         for (int t = 0; t < num_titans; t++)
             titans[t]->SetPIDType(1);
         usleep(40 * 1000);
     }
     usleep(150 * 1000);
     printf("  AutotuneAll() started, waiting 18 s for curve build...\n");
     for (int k = 0; k < 4; k++)
     {
         for (int t = 0; t < num_titans; t++)
             titans[t]->AutotuneAll();
         usleep(40 * 1000);
     }
     sleep(18);
 
     printf("  SetTargetVelocity all 4 -> %d rpm (PIDType 1)\n", (int)RPM_100);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 2, RPM_100, "100rpm");
     printf("  SetTargetVelocity all 4 -> -100 rpm (reverse, 5 s)\n");
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 5, -RPM_100, "100rpm rev");
     printf("  SetTargetVelocity all 4 -> %d rpm (continue)\n", (int)RPM_100);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 2, RPM_100, "100rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_50);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, HOLD_SEC, RPM_50, "50rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 1, RPM_0, "0rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_80);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, HOLD_SEC, RPM_80, "80rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 1, RPM_0, "0rpm");
 
     /* ---------- Phase 4: Sensitivity=10, SetTargetVelocity same speed sequence ---------- */
     printf("\n--- Phase 4: SetSensitivity(0..3, 10), SetTargetVelocity (same sequence) ---\n");
     for (int t = 0; t < num_titans; t++)
     {
         titans[t]->SetSensitivity(0, 10);
         titans[t]->SetSensitivity(1, 10);
         titans[t]->SetSensitivity(2, 10);
         titans[t]->SetSensitivity(3, 10);
     }
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_100);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, HOLD_SEC, RPM_100, "100rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_50);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, HOLD_SEC, RPM_50, "50rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 1, RPM_0, "0rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_80);
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, HOLD_SEC, RPM_80, "80rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titans, num_titans, 1, RPM_0, "0rpm");
 
     printf("\n--- Stop & Disable ---\n");
     stop_all(titans, num_titans);
     for (int i = 0; i < 3; i++)
     {
         for (int t = 0; t < num_titans; t++)
             titans[t]->Enable(false);
         usleep(80 * 1000);
     }
     usleep(400 * 1000);
     printf("Done.\n");
     return 0;
 }
 