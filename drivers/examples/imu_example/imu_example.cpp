#include "imu.h"
using namespace studica_driver;
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    studica_driver::Imu imu;
    for (int i = 0; i < 10; ++i) {
        printf("Yaw: %f, Pitch: %f, Roll: %f\n", imu.GetYaw(), imu.GetPitch(), imu.GetRoll());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}