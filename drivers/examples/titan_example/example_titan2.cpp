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
 * Build (Raspberry Pi, Titan2 root):
 *   g++ -o titan_speed_sequence_test VMX_HOST_TITAN_SPEED_SEQUENCE_TEST.cpp Host/titan.cpp \
 *       -I. -IHost -I/usr/local/include/vmxpi \
 *       -L/usr/local/frc/third-party/lib -lvmxpi_hal_cpp -lpthread -std=c++11
 *
 * Run: sudo ./titan_speed_sequence_test [CANID]
 *      e.g.: sudo ./titan_speed_sequence_test 20
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <unistd.h>
 #include "Host/titan.h"
 
/* Hold time per speed step (seconds) */
 static const int HOLD_SEC = 5;
 /* RPM sampling interval (ms); print RPM at this interval while motor runs */
 static const int RPM_PRINT_INTERVAL_MS = 200;
 /* In Phase 1 with SetSpeed, how often to resend SET_MOTOR_SPEED (firmware needs periodic receipt to maintain output) */
 static const int SET_SPEED_RESEND_MS = 50;

 /* SetTargetVelocity target RPM: 100, 50, 0, 80 (matches SetSpeed steps, unit rpm) */
 static const int16_t RPM_100 = 100;
 static const int16_t RPM_50  = 50;
 static const int16_t RPM_80  = 80;
 static const int16_t RPM_0   = 0;
 
 /* Phase 2/3/4: periodically resend SetTargetVelocity(0..3, target_rpm) so target changes (100→50→0→80→0) take effect, and print RPM every 200ms */
 static const int TARGET_VELOCITY_RESEND_MS = 50;
 
 static void run_for_seconds_target_velocity_and_rpm(studica_driver::Titan& titan, int seconds_sec, int16_t target_rpm, const char* label)
 {
     int total_ms = seconds_sec * 1000;
     int elapsed_ms = 0;
     int next_print_ms = 0;
     while (elapsed_ms < total_ms)
     {
         titan.SetTargetVelocity(0, target_rpm);
         titan.SetTargetVelocity(1, target_rpm);
         titan.SetTargetVelocity(2, target_rpm);
         titan.SetTargetVelocity(3, target_rpm);
         int sleep_ms = TARGET_VELOCITY_RESEND_MS;
         if (elapsed_ms + sleep_ms > total_ms) sleep_ms = total_ms - elapsed_ms;
         usleep(sleep_ms * 1000);
         elapsed_ms += sleep_ms;
         if (elapsed_ms >= next_print_ms)
         {
             next_print_ms += RPM_PRINT_INTERVAL_MS;
             usleep(8 * 1000);
             int16_t r0, r1, r2, r3;
             bool ok0 = titan.TryGetRPM(0, &r0);
             bool ok1 = titan.TryGetRPM(1, &r1);
             bool ok2 = titan.TryGetRPM(2, &r2);
             bool ok3 = titan.TryGetRPM(3, &r3);
             printf("  [%s] encoder RPM: M0=%d%c M1=%d%c M2=%d%c M3=%d%c\n",
                    label, (int)r0, ok0 ? ' ' : '?', (int)r1, ok1 ? ' ' : '?', (int)r2, ok2 ? ' ' : '?', (int)r3, ok3 ? ' ' : '?');
         }
     }
 }
 
 /* Stop: use only SetSpeed(motor,0), order 3→2→1→0 to avoid [0,0,0,0] being treated as legacy and stopping only ch0.
  * Do not send SetTargetVelocity(0) in Phase 1, otherwise HandleSetVelocity(0) will change device state and keep encoder RPM at 0. */
 static void stop_all(studica_driver::Titan& titan)
 {
     titan.SetSpeed(3, 0.0);
     titan.SetSpeed(2, 0.0);
     titan.SetSpeed(1, 0.0);
     titan.SetSpeed(0, 0.0);
     usleep(150 * 1000);
 }
 
 /* Phase 1: resend SetSpeed(0..3, duty) every SET_SPEED_RESEND_MS; duty<0 is reverse, duty=0 uses SetSpeed only to stop; print RPM every 200ms */
 static void run_for_seconds_set_speed_and_rpm(studica_driver::Titan& titan, int seconds_sec, double duty, const char* label)
 {
     int total_ms = seconds_sec * 1000;
     int elapsed_ms = 0;
     int next_print_ms = 0;
     while (elapsed_ms < total_ms)
     {
         if (duty == 0.0)
         {
             /* Order 3→2→1→0; Phase 1 uses SetSpeed only to stop, do not send SetTargetVelocity(0) to avoid affecting encoder */
             titan.SetSpeed(3, 0.0);
             titan.SetSpeed(2, 0.0);
             titan.SetSpeed(1, 0.0);
             titan.SetSpeed(0, 0.0);
         }
         else if (duty < 0.0)
         {
             titan.SetSpeed(0, duty);
             titan.SetSpeed(1, duty);
             titan.SetSpeed(2, duty);
             titan.SetSpeed(3, duty);
         }
         else
             titan.SetSpeedAll(duty);
         int sleep_ms = SET_SPEED_RESEND_MS;
         if (elapsed_ms + sleep_ms > total_ms) sleep_ms = total_ms - elapsed_ms;
         usleep(sleep_ms * 1000);
         elapsed_ms += sleep_ms;
         if (elapsed_ms >= next_print_ms)
         {
             next_print_ms += RPM_PRINT_INTERVAL_MS;
             /* Wait before read to avoid contending with SetSpeed on bus; let device send one round of sensor first */
             usleep(8 * 1000);
             int16_t r0, r1, r2, r3;
             bool ok0 = titan.TryGetRPM(0, &r0);
             bool ok1 = titan.TryGetRPM(1, &r1);
             bool ok2 = titan.TryGetRPM(2, &r2);
             bool ok3 = titan.TryGetRPM(3, &r3);
             printf("  [%s] encoder RPM: M0=%d%c M1=%d%c M2=%d%c M3=%d%c\n",
                    label, (int)r0, ok0 ? ' ' : '?', (int)r1, ok1 ? ' ' : '?', (int)r2, ok2 ? ' ' : '?', (int)r3, ok3 ? ' ' : '?');
         }
     }
 }
 
 int main(int argc, char** argv)
 {
     uint8_t canId = 42;
     if (argc >= 2)
         canId = (uint8_t)atoi(argv[1]);
 
     printf("=== Titan2 Speed Sequence Test (4 motors) ===\n");
     printf("CAN ID: %u\n", canId);
 
     studica_driver::Titan titan(canId, 2000, 1.0f);
     titan.Enable(true);
     sleep(1);
 
    /* Set PIDType 0 first, stop and clear target, then run SetSpeed */
    titan.SetPIDType(0);
    stop_all(titan);
    /* Wait for device to send several rounds of encoder/RPM before reading, to avoid Blackboard having no data yet */
    usleep(80 * 1000);

    /* Diagnostics: confirm device response and check if RPM frames are received */
     {
         uint8_t devId = titan.GetID();
         std::string fw = titan.GetFirmwareVersion();
         printf("  Device ID (from GET_TITAN_INFO): %u, Firmware: %s\n", (unsigned)devId, fw.c_str());
         int ok = 0;
         for (int i = 0; i < 15; i++)
         {
             int16_t r = 0;
             if (titan.TryGetRPM(0, &r))
                 ok++;
             usleep(15 * 1000);
         }
         printf("  RPM0 frames received (before Phase 1): %d/15\n", ok);
         if (ok == 0)
             printf("  >>> No RPM frames. Check: (1) Device CAN ID matches %u (2) Device sending sensor data every 10ms.\n", (unsigned)canId);
     }
 
     /* ---------- Phase 1: SetSpeed 100 -> 50 -> 0 -> 80 -> 0 (periodically resend SET_MOTOR_SPEED; when duty=0 also send target 0 to stop) ---------- */
     printf("\n--- Phase 1: SetPIDType(0), SetSpeed (duty 100%%, 50%%, 0, 80%%, 0) ---\n");
 
     printf("  SetSpeed all -> 100%% (resend every %d ms)\n", SET_SPEED_RESEND_MS);
     run_for_seconds_set_speed_and_rpm(titan, 2, 1.0, "100%");
     printf("  SetSpeed all -> 100%% reverse (5 s)\n");
     run_for_seconds_set_speed_and_rpm(titan, 5, -1.0, "100% rev");
     printf("  SetSpeed all -> 100%% (continue)\n");
     run_for_seconds_set_speed_and_rpm(titan, 2, 1.0, "100%");
 
     printf("  SetSpeed all -> 50%%\n");
     run_for_seconds_set_speed_and_rpm(titan, HOLD_SEC, 0.5, "50%");
 
     printf("  SetSpeed all -> 0 (duty + target 0 to stop)\n");
     stop_all(titan);
     run_for_seconds_set_speed_and_rpm(titan, 2, 0.0, "0");
 
     printf("  SetSpeed all -> 80%%\n");
     run_for_seconds_set_speed_and_rpm(titan, HOLD_SEC, 0.8, "80%");
 
     printf("  SetSpeed all -> 0 (duty + target 0 to stop)\n");
     stop_all(titan);
     run_for_seconds_set_speed_and_rpm(titan, 2, 0.0, "0");
 
    /* Wait for device to process remaining SetTargetVelocity(0) in queue before sending 100, to avoid target being overwritten back to 0 */
    usleep(150 * 1000);

    /* ---------- Phase 2: PIDType 0, SetTargetVelocity all 4 channels same target RPM 100->50->0->80->0 ---------- */
     printf("\n--- Phase 2: SetPIDType(0), SetTargetVelocity all 4 (target RPM 100->50->0->80->0) ---\n");
     titan.SetPIDType(0);
 
    /* At start send multiple rounds of 100 to fill queue, avoid late (0) overwriting target */
    for (int i = 0; i < 8; i++)
     {
         titan.SetTargetVelocity(0, RPM_100);
         titan.SetTargetVelocity(1, RPM_100);
         titan.SetTargetVelocity(2, RPM_100);
         titan.SetTargetVelocity(3, RPM_100);
         usleep(10 * 1000);
     }
 
    /* Read back device target speed to confirm SET_TARGET_VELOCITY was received */
    {
         int16_t devTarget[4] = {0, 0, 0, 0};
         if (titan.GetTargetRPMFromDevice(devTarget))
             printf("  Device target RPM (readback): M0=%d M1=%d M2=%d M3=%d\n",
                    (int)devTarget[0], (int)devTarget[1], (int)devTarget[2], (int)devTarget[3]);
         else
             printf("  Device target RPM readback failed.\n");
     }
    /* Give device time for PID output to take effect and encoder to have non-zero values before printing */
    usleep(150 * 1000);
 
     printf("  SetTargetVelocity all 4 -> %d rpm (resend every %d ms)\n", (int)RPM_100, TARGET_VELOCITY_RESEND_MS);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_100, "100rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_50);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_50, "50rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titan, 1, RPM_0, "0rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_80);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_80, "80rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titan, 1, RPM_0, "0rpm");
 
    /* ---------- Phase 3: PIDType 1 requires Autotune first, then SetTargetVelocity ---------- */
    printf("\n--- Phase 3: SetPIDType(1), AutotuneAll(), then SetTargetVelocity (same sequence) ---\n");
    /* Enable again and send one round of SetTargetVelocity(0) to ensure device USBOverride=1 and enabled, then send PIDType and Autotune */
     titan.Enable(true);
     usleep(50 * 1000);
     titan.SetTargetVelocity(0, 0);
     titan.SetTargetVelocity(1, 0);
     titan.SetTargetVelocity(2, 0);
     titan.SetTargetVelocity(3, 0);
     usleep(100 * 1000);
     for (int k = 0; k < 4; k++)
     {
         titan.SetPIDType(1);
         usleep(40 * 1000);
     }
     usleep(150 * 1000);
     printf("  AutotuneAll() started, waiting 18 s for curve build...\n");
     for (int k = 0; k < 4; k++)
     {
         titan.AutotuneAll();
         usleep(40 * 1000);
     }
     sleep(18);
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_100);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_100, "100rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_50);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_50, "50rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titan, 1, RPM_0, "0rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_80);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_80, "80rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titan, 1, RPM_0, "0rpm");
 
     /* ---------- Phase 4: Sensitivity=10, SetTargetVelocity same speed sequence ---------- */
     printf("\n--- Phase 4: SetSensitivity(0..3, 10), SetTargetVelocity (same sequence) ---\n");
     titan.SetSensitivity(0, 10);
     titan.SetSensitivity(1, 10);
     titan.SetSensitivity(2, 10);
     titan.SetSensitivity(3, 10);
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_100);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_100, "100rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_50);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_50, "50rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titan, 1, RPM_0, "0rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_80);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_80, "80rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titan, 1, RPM_0, "0rpm");
 
     printf("\n--- Stop & Disable ---\n");
     stop_all(titan);
     titan.Enable(false);
     printf("Done.\n");
     return 0;
 }
 