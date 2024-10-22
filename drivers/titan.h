#ifndef TITAN_H
#define TITAN_H

#include <stdio.h>  
#include <string.h>
#include <inttypes.h>
#include <sstream>
#include <iomanip>
#include <memory>
#include "VMXPi.h"
 
namespace studica_driver
{

#define ID                         42
#define DEVICE_TYPE                33554432
#define MANUFACTURER_ID            786432
#define OFFSET                     64
#define BASE                       DEVICE_TYPE + MANUFACTURER_ID + ID
 
#define DISABLED_FLAG              BASE
#define ENABLED_FLAG               BASE + (OFFSET * 1)
#define SET_MOTOR_SPEED            BASE + (OFFSET * 2)
#define DISABLE_MOTOR              BASE + (OFFSET * 3)
#define GET_TITAN_INFO             BASE + (OFFSET * 4)
#define RETURN_TITAN_INFO          BASE + (OFFSET * 5)
#define GET_UNIQUE_ID              BASE + (OFFSET * 6)
#define RETURN_WORD_1              BASE + (OFFSET * 7)
#define RETURN_WORD_2              BASE + (OFFSET * 8)
#define RETURN_WORD_3              BASE + (OFFSET * 9)
#define CONFIG_MOTOR               BASE + (OFFSET * 10)
#define GET_MOTOR_FREQUENCY        BASE + (OFFSET * 11)
#define RETURN_MOTOR_FREQUENCY     BASE + (OFFSET * 12)
#define RESET_ENCODER              BASE + (OFFSET * 13)
#define SET_CURRENT_LIMIT          BASE + (OFFSET * 14)
#define SET_MOTOR_STOP_MODE        BASE + (OFFSET * 15)
#define SET_TARGET_VELOCITY        BASE + (OFFSET * 16)
#define SET_TARGET_DISTANCE        BASE + (OFFSET * 17)
#define RETURN_PID_TARGET_STATUS   BASE + (OFFSET * 19)
#define SET_PID_VALUES             BASE + (OFFSET * 20)
#define SET_PID_LIMITS             BASE + (OFFSET * 21)
#define CYPHER_OUTPUT              BASE + (OFFSET * 36)
#define ENCODER_0_COUNT            BASE + (OFFSET * 37)
#define ENCODER_1_COUNT            BASE + (OFFSET * 38)
#define ENCODER_2_COUNT            BASE + (OFFSET * 39)
#define ENCODER_3_COUNT            BASE + (OFFSET * 40)
#define RPM_0                      BASE + (OFFSET * 41)
#define RPM_1                      BASE + (OFFSET * 42)
#define RPM_2                      BASE + (OFFSET * 43)
#define RPM_3                      BASE + (OFFSET * 44)
#define LIMIT_SWITCH               BASE + (OFFSET * 45)
#define CURRENT_VALUE              BASE + (OFFSET * 46)
#define MCU_TEMP                   BASE + (OFFSET * 47)

class Titan
{
    public:
        Titan(const std::string &name, const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, const float &speed, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
        ~Titan();
        void Enable(bool enable);
        void SetupEncoder(uint8_t encoder);
        uint8_t GetID();
        uint16_t GetFrequency();
        std::string GetFirmwareVersion();
        std::string GetHardwareVersion();
        float GetControllerTemp();
        bool GetLimitSwitch(uint8_t motor, uint8_t direction);
        int16_t GetRPM(uint8_t motor);
        std::string GetSerialNumber();
        double GetEncoderDistance(uint8_t motor);
        int32_t GetEncoderCount(uint8_t motor);
        void ConfigureEncoder(uint8_t motor, double cfg);
        void ResetEncoder(uint8_t motor);
        double GetCypherAngle(uint8_t port);
        void SetSpeed(uint8_t motor, double speedCfg);
        void InvertMotorDirection(uint8_t motor);
        void InvertMotorRPM(uint8_t motor);
        void InvertEncoderDirection(uint8_t motor);
 
    private:
        std::shared_ptr<VMXPi> vmx_;
        uint8_t canID_;
        uint16_t motorFreq_;
        uint8_t nEncoder_;
        float distPerTick_;
        float speed_;

        VMXCANReceiveStreamHandle canrxhandle = 0;
        VMXErrorCode vmxerr;
        bool Write(uint32_t address, const uint8_t* data, int32_t periodMS);
        bool Read(uint32_t address, uint8_t* data);
        void DisplayVMXError();
        // Motor Flags
        bool invertMotor0 = false;
        bool invertRPM0 = false;
        bool invertEncoder0 = false;
        bool invertMotor1 = false;
        bool invertRPM1 = false;
        bool invertEncoder1 = false;
        bool invertMotor2 = false;
        bool invertRPM2 = false;
        bool invertEncoder2 = false;
        bool invertMotor3 = false;
        bool invertRPM3 = false;
        bool invertEncoder3 = false;
 
        //Encoder stuff
        double distPerTick_0 = 0;
        double distPerTick_1 = 0;
        double distPerTick_2 = 0;
        double distPerTick_3 = 0;
 
};

} // namespace studica_driver

#endif // TITAN_H