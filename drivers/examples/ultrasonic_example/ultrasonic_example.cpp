#include "ultrasonic.h"
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    VMXChannelIndex ping = 8;
    VMXChannelIndex echo = 9;
    studica_driver::Ultrasonic ultrasonic(ping, echo);

    for (int i = 0; i < 100; i++) {
        ultrasonic.Ping();
        std::this_thread::sleep_for(std::chrono::milliseconds(55));
        printf("Distance (inches): %.2f\n", ultrasonic.GetDistanceIN());
    }
    for (int i = 0; i < 100; i++) {
        ultrasonic.Ping();
        std::this_thread::sleep_for(std::chrono::milliseconds(55));
        printf("Distance (mm): %.2f\n", ultrasonic.GetDistanceMM());
    }
    return 0;
}