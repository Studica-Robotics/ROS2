#include "servo.h"
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    VMXChannelIndex port = 13; // servo output on pin 14
    studica_driver::Servo servo(port, studica_driver::ServoType::Standard, -150, 150);  
    servo.SetAngle(150);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetAngle(-150);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetAngle(0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return 0;
}