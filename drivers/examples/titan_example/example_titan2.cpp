/*
 * VMX-Pi Host: Titan2 四通道速度序列测试
 *
 * 流程：
 *   1. SetSpeed 四路同占空比: 100% -> 50% -> 0 -> 80% -> 0 (duty cycle %)
 *   2. SetPIDType(0)，SetTargetVelocity 四路同目标 RPM: 100 -> 50 -> 0 -> 80 -> 0
 *   3. SetPIDType(1)，同上目标 RPM 再来一遍
 *   4. SetSensitivity(0..3, 10)，同上目标 RPM 再来一遍
 * 电机转动期间持续输出 encoder 读到的 RPM。
 *
 * 编译（树莓派，Titan2 根目录）:
 *   g++ -o titan_speed_sequence_test VMX_HOST_TITAN_SPEED_SEQUENCE_TEST.cpp Host/titan.cpp \
 *       -I. -IHost -I/usr/local/include/vmxpi \
 *       -L/usr/local/frc/third-party/lib -lvmxpi_hal_cpp -lpthread -std=c++11
 *
 * 运行: sudo ./titan_speed_sequence_test [CANID]
 *       例如: sudo ./titan_speed_sequence_test 20
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <unistd.h>
 #include "Host/titan.h"
 
 static studica_driver::Titan* s_titan_for_atexit = nullptr;
 static void atexit_disable_titan(void)
 {
     if (s_titan_for_atexit)
     {
         s_titan_for_atexit->Enable(false);
         usleep(400 * 1000);
     }
 }
 
 /* 每个速度档位保持时间 (秒) */
 static const int HOLD_SEC = 5;
 /* 转速采样间隔 (毫秒)，电机转时按此间隔打印 RPM */
 static const int RPM_PRINT_INTERVAL_MS = 200;
 /* Phase 1 用 SetSpeed 时，每隔多久重发一次 SET_MOTOR_SPEED。须 < TITAN_CAN_KEEPALIVE_MS(150) 以免设备 200ms 超时 disable。 */
 static const int SET_SPEED_RESEND_MS = 50;
 
 /* SetTargetVelocity 目标 RPM：100, 50, 0, 80（与 SetSpeed 档位对应，单位 rpm） */
 static const int16_t RPM_100 = 100;
 static const int16_t RPM_50  = 50;
 static const int16_t RPM_80  = 80;
 static const int16_t RPM_0   = 0;
 
 /* Phase 2/3/4 专用：周期重发 SetTargetVelocity。须 < TITAN_CAN_KEEPALIVE_MS 以免设备 200ms 超时；并每 200ms 打印 RPM */
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
 
 /* 停转：SetSpeedAll(0) 发 4 帧 [motor, 0, 1, 0] 停四路。不在 Phase 1 发 SetTargetVelocity(0)，否则会改设备状态。 */
 static void stop_all(studica_driver::Titan& titan)
 {
     titan.SetSpeedAll(0.0);
     usleep(150 * 1000);
 }
 
 /* Phase 1 专用：每 SET_SPEED_RESEND_MS 重发 SET_MOTOR_SPEED（Titan 格式一帧一电机）；duty<0 为反转，duty=0 用 SetSpeedAll(0) 停转；每 200ms 打印 RPM */
 static void run_for_seconds_set_speed_and_rpm(studica_driver::Titan& titan, int seconds_sec, double duty, const char* label)
 {
     int total_ms = seconds_sec * 1000;
     int elapsed_ms = 0;
     int next_print_ms = 0;
     while (elapsed_ms < total_ms)
     {
         if (duty == 0.0)
         {
             titan.SetSpeedAll(0.0);
         }
         else if (duty < 0.0)
         {
             titan.SetSpeed(0, duty);
             usleep(3 * 1000);
             titan.SetSpeed(1, duty);
             usleep(3 * 1000);
             titan.SetSpeed(2, duty);
             usleep(3 * 1000);
             titan.SetSpeed(3, duty);
         }
         else
         {
             titan.SetSpeedAll(duty);
         }
         int sleep_ms = SET_SPEED_RESEND_MS;
         if (elapsed_ms + sleep_ms > total_ms) sleep_ms = total_ms - elapsed_ms;
         usleep(sleep_ms * 1000);
         elapsed_ms += sleep_ms;
         if (elapsed_ms >= next_print_ms)
         {
             next_print_ms += RPM_PRINT_INTERVAL_MS;
             /* 读前稍等，避免和 SetSpeed 同一时刻抢总线，让设备先发一轮 sensor */
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
     s_titan_for_atexit = &titan;
     atexit(atexit_disable_titan);
     titan.Enable(true);   /* Device powers up disabled; must send ENABLED_FLAG before motor commands */
     sleep(1);
 
     /* 先设 PIDType 0，停转清空目标，再跑 SetSpeed */
     titan.SetPIDType(0);
     stop_all(titan);
     /* 等设备发几轮 encoder/RPM 再读，避免 Blackboard 尚无数据 */
     usleep(80 * 1000);
 
     /* 诊断：确认能收到设备应答，并检查是否收到 RPM 帧 */
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
 
     /* ---------- Phase 1: SetSpeed 100 -> 50 -> 0 -> 80 -> 0（周期重发 SET_MOTOR_SPEED，Titan 格式一帧一电机；duty=0 时 SetSpeedAll(0) 停转）---------- */
     printf("\n--- Phase 1: SetPIDType(0), SetSpeed (duty 100%%, 50%%, 0, 80%%, 0) ---\n");
 
     printf("  SetSpeed all -> 100%% (resend every %d ms)\n", SET_SPEED_RESEND_MS);
     run_for_seconds_set_speed_and_rpm(titan, 2, 1.0, "100%");
     printf("  SetSpeed all -> 100%% reverse (5 s)\n");
     run_for_seconds_set_speed_and_rpm(titan, 5, -1.0, "100% rev");
     printf("  SetSpeed all -> 100%% (continue)\n");
     run_for_seconds_set_speed_and_rpm(titan, 2, 1.0, "100%");
 
     printf("  SetSpeed all -> 50%%\n");
     run_for_seconds_set_speed_and_rpm(titan, HOLD_SEC, 0.5, "50%");
 
     printf("  SetSpeed all -> 0 (SetSpeedAll(0) to stop)\n");
     stop_all(titan);
     run_for_seconds_set_speed_and_rpm(titan, 2, 0.0, "0");
 
     printf("  SetSpeed all -> 80%%\n");
     run_for_seconds_set_speed_and_rpm(titan, HOLD_SEC, 0.8, "80%");
 
     printf("  SetSpeed all -> 0 (SetSpeedAll(0) to stop)\n");
     stop_all(titan);
     run_for_seconds_set_speed_and_rpm(titan, 2, 0.0, "0");
 
     /* 等设备把队列里残留的 SetTargetVelocity(0) 处理完，再发 100，避免 target 被覆盖回 0 */
     usleep(150 * 1000);
 
     /* ---------- Phase 2: PIDType 0, SetTargetVelocity 四路同目标 RPM 100->50->0->80->0 ---------- */
     printf("\n--- Phase 2: SetPIDType(0), SetTargetVelocity all 4 (target RPM 100->50->0->80->0) ---\n");
     titan.SetPIDType(0);
 
     /* 开局连发多轮 100，把队列灌满，避免迟到的 (0) 覆盖 target */
     for (int i = 0; i < 8; i++)
     {
         titan.SetTargetVelocity(0, RPM_100);
         titan.SetTargetVelocity(1, RPM_100);
         titan.SetTargetVelocity(2, RPM_100);
         titan.SetTargetVelocity(3, RPM_100);
         usleep(10 * 1000);
     }
 
     /* 读回设备内目标转速，确认 SET_TARGET_VELOCITY 是否被接收 */
     {
         int16_t devTarget[4] = {0, 0, 0, 0};
         if (titan.GetTargetRPMFromDevice(devTarget))
             printf("  Device target RPM (readback): M0=%d M1=%d M2=%d M3=%d\n",
                    (int)devTarget[0], (int)devTarget[1], (int)devTarget[2], (int)devTarget[3]);
         else
             printf("  Device target RPM readback failed.\n");
     }
     /* 给设备时间让 PID 输出生效、encoder 有非零值后再开始打印 */
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
 
     /* ---------- Phase 3: PIDType 1 需先 Autotune，再 SetTargetVelocity ---------- */
     printf("\n--- Phase 3: SetPIDType(1), AutotuneAll(), then SetTargetVelocity (same sequence) ---\n");
     /* 再次 Enable 并发一轮 SetTargetVelocity(0)，确保设备 USBOverride=1、已使能，再发 PIDType 和 Autotune */
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
 
     printf("  SetTargetVelocity all 4 -> %d rpm (PIDType 1)\n", (int)RPM_100);
     run_for_seconds_target_velocity_and_rpm(titan, 2, RPM_100, "100rpm");
     printf("  SetTargetVelocity all 4 -> -100 rpm (reverse, 5 s)\n");
     run_for_seconds_target_velocity_and_rpm(titan, 5, -RPM_100, "100rpm rev");
     printf("  SetTargetVelocity all 4 -> %d rpm (continue)\n", (int)RPM_100);
     run_for_seconds_target_velocity_and_rpm(titan, 2, RPM_100, "100rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_50);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_50, "50rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titan, 1, RPM_0, "0rpm");
 
     printf("  SetTargetVelocity all 4 -> %d rpm\n", (int)RPM_80);
     run_for_seconds_target_velocity_and_rpm(titan, HOLD_SEC, RPM_80, "80rpm");
 
     printf("  SetTargetVelocity all 4 -> 0\n");
     run_for_seconds_target_velocity_and_rpm(titan, 1, RPM_0, "0rpm");
 
     /* ---------- Phase 4: Sensitivity=10, SetTargetVelocity 同样速度 ---------- */
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
     for (int i = 0; i < 3; i++)
     {
         titan.Enable(false);
         usleep(80 * 1000);
     }
     usleep(400 * 1000);
     printf("Done.\n");
     return 0;
 }
 