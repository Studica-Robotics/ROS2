/**
 * Example host code for Parsec ToF sensor over CAN (VMXPi).
 * Targets a single device at CAN ID 5; no scanning. Optionally runs DATA stream (set DATA_LOOP_MS > 0).
 */

 #include "parsec.h"
 #include "VMXPi.h"
 #include <stdio.h>
 #include <cstring>
 #include <memory>
 
 using namespace studica_driver;
 
 #define PARSEC_TARGET_CAN_ID  5
 #define DATA_LOOP_MS          500     // 0 = no DATA stream; >0 = run stream this many ms
 
 static void print_config(uint8_t canID, const uint8_t* config, int len)
 {
     if (len < 8) return;
     printf("  [CAN ID %u] version=%u  fdist_en=%u  can_fd=%u  can_brs=%u  reported_id=%u\n",
            canID, config[0], config[1], config[2], config[3], config[4]);
     if (len >= 44) {
         uint64_t zoneMask = Parsec::ParseConfigZoneMask(config);
         printf("            zone_mask=0x%016llX\n", (unsigned long long)zoneMask);
     }
 }
 
 static void print_data_chunk(int chunkIndex, const uint8_t* data, int len)
 {
     if (len < 2) return;
     uint8_t seq = data[0], zones = data[1];
     printf("  DATA%d  seq=%u  zones=%u  len=%d\n", chunkIndex, seq, zones, len);
     int numDist = (len - 2) / 2;
     if (numDist > 8) numDist = 8;
     printf("  dist(mm): ");
     for (int i = 0; i < numDist; i++) {
         int16_t d = (int16_t)(data[2 + i*2] | (data[3 + i*2] << 8));
         printf("%d ", (int)d);
     }
     if (numDist * 2 < len - 2) printf("...");
     printf("\n");
 }
 
 static void print_can2_data(uint8_t seq, uint8_t zones, const int16_t* fdist, int count)
 {
     printf("  CAN2 DATA  seq=%u  zones=%u  count=%d\n", seq, zones, count);
     printf("  dist(mm): ");
     for (int i = 0; i < count; i++) printf("%d ", (int)fdist[i]);
     printf("\n");
 }
 
 int main(int argc, char** argv)
 {
     (void)argc;
     (void)argv;
 
     printf("Parsec example: target CAN ID %d\n", PARSEC_TARGET_CAN_ID);
     fflush(stdout);
 
     printf("Opening VMXPi ...\n");
     fflush(stdout);
     std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);
     if (!vmx->IsOpen()) {
         printf("VMXPi failed to open. (pigpio/root?)\n");
         fflush(stdout);
         return 1;
     }
     printf("VMXPi open OK.\n");
     fflush(stdout);
 
     Parsec parsec(PARSEC_TARGET_CAN_ID, vmx);
     if (parsec.GetCanID() != PARSEC_TARGET_CAN_ID) {
         printf("Parsec init failed.\n");
         fflush(stdout);
         return 1;
     }
 
     uint8_t req[8] = {0, 0, 0, 0, 0, 0, 0, 0};
     uint8_t config[64];
 
     printf("--- GETCONFIG to CAN ID %d, drain 12 ms x 8 ---\n", PARSEC_TARGET_CAN_ID);
     fflush(stdout);
     (void)parsec.Write(parsec.GetAddress(PARSEC_GETCONFIG), req, 8, 0);
     for (int d = 0; d < 8; d++) {
         vmx->time.DelayMilliseconds(12);
         parsec.PullCANData();
     }
     int n0 = parsec.ReadCached(parsec.GetAddress(PARSEC_CMD_RESP), config, sizeof(config));
     if (n0 < 8) {
         printf("No response from CAN ID %d (check bus, device CAN ID setting).\n", PARSEC_TARGET_CAN_ID);
         fflush(stdout);
         return 0;
     }
     printf("  found CAN ID %d\n", PARSEC_TARGET_CAN_ID);
     print_config((uint8_t)PARSEC_TARGET_CAN_ID, config, n0 >= 44 ? 44 : n0);
 
     if (DATA_LOOP_MS > 0) {
         uint8_t cfg[64];
         (void)parsec.Write(parsec.GetAddress(PARSEC_GETCONFIG), req, 8, 0);
         vmx->time.DelayMilliseconds(30);
         bool have_cfg = parsec.GetConfigResponse(cfg, sizeof(cfg));
         bool can_fd = have_cfg && cfg[2] != 0;
         int16_t fdist[64];
         uint8_t seq = 0, zones = 0;
         int n_can2 = parsec.ReadDataStreamCAN2(&seq, &zones, fdist, 64);
         bool use_can2 = (n_can2 >= 16 || (n_can2 > 0 && !can_fd));
         if (use_can2) can_fd = false;
         printf("\n--- DATA stream (CAN ID %d, %d ms) %s ---\n", PARSEC_TARGET_CAN_ID, DATA_LOOP_MS, can_fd ? "CAN FD" : "CAN2");
         const int interval_ms = 100;
         int iterations = DATA_LOOP_MS / interval_ms;
         if (iterations <= 0) iterations = 1;
         for (int it = 0; it < iterations; it++) {
             if (can_fd) {
                 for (int c = 0; c < 4; c++) {
                     uint8_t chunk[64];
                     int n = parsec.ReadDataChunk((uint32_t)c, chunk, sizeof(chunk));
                     if (n >= 2) print_data_chunk(c, chunk, n);
                 }
             } else {
                 int n = parsec.ReadDataStreamCAN2(&seq, &zones, fdist, 64);
                 if (n > 0) print_can2_data(seq, zones, fdist, n);
             }
             vmx->time.DelayMilliseconds(interval_ms);
         }
     }
 
     printf("\nDone.\n");
     fflush(stdout);
     return 0;
 }
 