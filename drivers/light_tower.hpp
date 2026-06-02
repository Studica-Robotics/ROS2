#pragma once

#include "VMXPi.h"
#include "dio.hpp"
#include <memory>

namespace studica_driver {

/*
 * LightTower — thin hardware abstraction for a 5-pin LED tower.
 *
 * Pin wiring:
 *   continuous — HIGH = solid/on, LOW = hardware blinks automatically
 *   red        — HIGH = red segment on
 *   green      — HIGH = green segment on
 *   yellow     — HIGH = yellow segment on
 *   buzzer     — HIGH = buzzer on
 *
 * Only one of red/green/yellow/buzzer should be active at once.
 * AllOff() clears everything including the continuous pin.
 */
class LightTower {
public:
    LightTower(VMXChannelIndex pin_continuous,
               VMXChannelIndex pin_red,
               VMXChannelIndex pin_green,
               VMXChannelIndex pin_yellow,
               VMXChannelIndex pin_buzzer,
               std::shared_ptr<VMXPi> vmx);
    ~LightTower();

    void SetContinuous(bool value);
    void SetRed(bool value);
    void SetGreen(bool value);
    void SetYellow(bool value);
    void SetBuzzer(bool value);
    void AllOff();

private:
    DIO continuous_;
    DIO red_;
    DIO green_;
    DIO yellow_;
    DIO buzzer_;
};

} // namespace studica_driver
