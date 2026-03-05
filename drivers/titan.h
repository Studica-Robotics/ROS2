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

/* CAN address = BASE + canID + (OFFSET * index). GetAddress(index) at runtime. */
#define TITAN_DEVICE_TYPE          33554432
#define TITAN_MANUFACTURER_ID      786432
#define TITAN_OFFSET               64
#define BASE                       (TITAN_DEVICE_TYPE + TITAN_MANUFACTURER_ID)
#define OFFSET                     TITAN_OFFSET

/* CAN API: address = BASE + canID + (OFFSET * index). Use GetAddress(XXX) to add canID. */
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
#define SET_ENCODER_RESOLUTION     BASE + (OFFSET * 18)
#define SET_CURRENT_LIMIT_MODE     BASE + (OFFSET * 19)
#define SET_PID_TYPE               BASE + (OFFSET * 20)
#define AUTOTUNE_ALL               BASE + (OFFSET * 21)
#define SET_SENSITIVITY            BASE + (OFFSET * 22)
#define SET_TARGET_ANGLE           BASE + (OFFSET * 23)
#define SET_POSITION_HOLD          BASE + (OFFSET * 24)
#define GET_TARGET_RPM             BASE + (OFFSET * 25)
#define CYPHER_OUTPUT              BASE + (OFFSET * 36)
#define ENCODER_0                  BASE + (OFFSET * 37)
#define ENCODER_1                  BASE + (OFFSET * 38)
#define ENCODER_2                  BASE + (OFFSET * 39)
#define ENCODER_3                  BASE + (OFFSET * 40)
#define CAN_RPM_0                  BASE + (OFFSET * 41)
#define CAN_RPM_1                  BASE + (OFFSET * 42)
#define CAN_RPM_2                  BASE + (OFFSET * 43)
#define CAN_RPM_3                  BASE + (OFFSET * 44)
#define LIMIT_SWITCH               BASE + (OFFSET * 45)
#define TARGET_RPM                 BASE + (OFFSET * 46)
#define MCU_TEMP                   BASE + (OFFSET * 47)

class Titan
{
    public:
        Titan(const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
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
        /** Returns true if a RPM frame was read; false if no frame (Blackboard empty for that ID). Fills *out_rpm. */
        bool TryGetRPM(uint8_t motor, int16_t* out_rpm);
        std::string GetSerialNumber();
        double GetEncoderDistance(uint8_t motor);
        int32_t GetEncoderCount(uint8_t motor);
        void ConfigureEncoder(uint8_t motor, double cfg);
        void ResetEncoder(uint8_t motor);
        double GetCypherAngle(uint8_t port);
        void SetSpeed(uint8_t motor, double speedCfg);
        /** Set all 4 channels to same duty (0..1) in one CAN frame. Use to avoid jitter from sending 4 frames. */
        void SetSpeedAll(double duty);
        void InvertMotorDirection(uint8_t motor);
        void InvertMotorRPM(uint8_t motor);
        void InvertEncoderDirection(uint8_t motor);
        void InvertMotor(uint8_t motor);

        /* Titan2 extended CAN API */
        /** Set target RPM; negative = reverse. PID (or MCV2) drives motor via setMotorSpeed(..., inA, inB). */
        void SetTargetVelocity(uint8_t motor, int16_t velocityRpm);
        /** Read back current velocity targets from device (GET_TARGET_RPM). Returns true if read ok. */
        bool GetTargetRPMFromDevice(int16_t targetRpm[4]);
        void SetTargetDistance(uint8_t motor, int32_t distanceCounts);
        void SetTargetAngle(uint8_t motor, double angleDeg);
        void SetPositionHold(uint8_t motor, bool hold);
        void SetEncoderResolution(uint8_t channel, uint16_t cpr);
        void SetCurrentLimit(uint8_t channel, float limitAmps);
        void SetCurrentLimitMode(uint8_t channel, uint8_t mode);
        void SetMotorStopMode(uint8_t mode);
        void SetPIDType(uint8_t type);
        void AutotuneAll();
        void SetSensitivity(uint8_t motor, uint8_t sensitivity);
        void DisableMotor(uint8_t motor);

    private:
        std::shared_ptr<VMXPi> vmx_;
        uint8_t canID_;
        uint16_t motorFreq_;
        uint8_t nEncoder_;
        float distPerTick_;

        /** Build CAN ID for this device: addressBase + canID_. addressBase = BASE + (OFFSET * n). */
        uint32_t GetAddress(uint32_t addressBase) const;

        VMXCANReceiveStreamHandle canrxhandle = 0;
        VMXErrorCode vmxerr;
        bool Write(uint32_t address, const uint8_t* data, int32_t periodMS);
        bool Read(uint32_t address, uint8_t* data);
        /** Cache RETURN_TITAN_INFO so GetID/GetFirmwareVersion/GetHardwareVersion share one read (VMX Blackboard may only serve one consume per ID). */
        uint8_t cached_titan_info_[8] = {0};
        bool cached_titan_info_valid_ = false;
        bool EnsureTitanInfoCached();
        /** Titan2 SET_MOTOR_SPEED: one frame = data[0..3] = ch0..ch3 duty 0-100%, data[4] = direction bits (bit i: 1=forward, 0=reverse). */
        uint8_t lastDuty_[4] = {0, 0, 0, 0};
        uint8_t lastDirection_ = 0x0F;   /* all forward */
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