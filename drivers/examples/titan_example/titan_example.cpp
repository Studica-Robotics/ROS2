#include "titan.h"
#include <thread>
#include <chrono>
#include <iostream>

int main(int argc, char *argv[])
{
    studica_driver::Titan titan("ttn1", (uint8_t)45, (uint16_t)15600, (float)0.0006830601, (float)0.8);
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(1.0 * 1000))); //Req after config of titan

    printf("Serial Number: %s\n", titan.GetSerialNumber().c_str());
    printf("Firmware Version: %s\n", titan.GetFirmwareVersion().c_str());
    printf("Hardware Version: %s\n", titan.GetHardwareVersion().c_str());
    
    titan.ConfigureEncoder(0, 0.0006830601); //1464 = 1 rotation
    titan.ConfigureEncoder(1, 0.0006830601);
    titan.ConfigureEncoder(2, 0.0006830601);
    titan.ConfigureEncoder(3, 0.0006830601);
    
    titan.ResetEncoder(0);
    titan.ResetEncoder(1);
    titan.ResetEncoder(2);
    titan.ResetEncoder(3);

    // These are flags to be uncommented as needed
    // titan.InvertEncoderDirection(0);
    // titan.InvertEncoderDirection(1);
    // titan.InvertEncoderDirection(2);
    // titan.InvertEncoderDirection(3);
 
    // titan.InvertMotorDirection(0);
    // titan.InvertMotorDirection(1);
    // titan.InvertMotorDirection(2);
    // titan.InvertMotorDirection(3);
 
    // titan.InvertMotorRPM(0);
    // titan.InvertMotorRPM(1);
    // titan.InvertMotorRPM(2);
    // titan.InvertMotorRPM(3);

    titan.Enable(true);
    
    for(int i = 0; i < 5; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(2.0 * 1000)));
        titan.SetSpeed(0, -0.2);
        titan.SetSpeed(1, 0.2);
        titan.SetSpeed(2, -0.2);
        titan.SetSpeed(3, 0.2);
        printf("Encoder 0 Distance: %f, RPM: %d, Raw: %i\n", titan.GetEncoderDistance(0), titan.GetRPM(0), titan.GetEncoderCount(0));
        printf("Encoder 1 Distance: %f, RPM: %d, Raw: %i\n", titan.GetEncoderDistance(1), titan.GetRPM(1), titan.GetEncoderCount(1));
        printf("Encoder 2 Distance: %f, RPM: %d, Raw: %i\n", titan.GetEncoderDistance(2), titan.GetRPM(2), titan.GetEncoderCount(2));
        printf("Encoder 3 Distance: %f, RPM: %d, Raw: %i\n", titan.GetEncoderDistance(3), titan.GetRPM(3), titan.GetEncoderCount(3));
    }

    titan.Enable(false);
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(5.0 * 1000))); //This Delay is just to check if the Titan does disable
}

