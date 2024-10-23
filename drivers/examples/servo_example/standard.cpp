#include "servo.h"
#include <thread>
#include <chrono>
#define SERVO_PIN 13

int main(int argc, char *argv[])
{
    VMXChannelIndex port = SERVO_PIN;
    studica_driver::Servo servo(port, studica_driver::ServoType::Standard, -150, 150);  
    servo.SetAngle(150);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetAngle(-150);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetAngle(0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return 0;
}