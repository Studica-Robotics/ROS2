#include "sharp.h"
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    VMXChannelIndex port = 22; // either one of 22~25
    studica_driver::Sharp sharp(port);
    for (int i = 0; i < 10; ++i) {
        printf("%d |  Voltage: %f, ", i, sharp.GetVoltage());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        printf("Distance: %f\n", sharp.GetDistance());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}