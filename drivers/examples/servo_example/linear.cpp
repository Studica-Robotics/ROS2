#include "servo.h"
#include <thread>
#include <chrono>

#define SERVO_PIN 12
int main(int argc, char *argv[])
{
    VMXChannelIndex port = SERVO_PIN;
    studica_driver::Servo servo(port, studica_driver::ServoType::Linear, 0, 100);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    servo.SetBounds(1, 1.5, 2);
    servo.SetAngle(0);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    servo.SetAngle(100);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    servo.SetAngle(50);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return 0;
}