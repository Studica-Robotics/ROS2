#include "light_tower.hpp"
using namespace studica_driver;

LightTower::LightTower(VMXChannelIndex pin_continuous,
                       VMXChannelIndex pin_red,
                       VMXChannelIndex pin_green,
                       VMXChannelIndex pin_yellow,
                       VMXChannelIndex pin_buzzer,
                       std::shared_ptr<VMXPi> vmx)
    : continuous_(pin_continuous, PinMode::OUTPUT, vmx)
    , red_       (pin_red,        PinMode::OUTPUT, vmx)
    , green_     (pin_green,      PinMode::OUTPUT, vmx)
    , yellow_    (pin_yellow,     PinMode::OUTPUT, vmx)
    , buzzer_    (pin_buzzer,     PinMode::OUTPUT, vmx)
{
    AllOff();
}

LightTower::~LightTower()
{
    AllOff();
}

void LightTower::SetContinuous(bool value) { continuous_.Set(value); }
void LightTower::SetRed(bool value)        { red_.Set(value);        }
void LightTower::SetGreen(bool value)      { green_.Set(value);      }
void LightTower::SetYellow(bool value)     { yellow_.Set(value);     }
void LightTower::SetBuzzer(bool value)     { buzzer_.Set(value);     }

void LightTower::AllOff()
{
    continuous_.Set(false);
    red_.Set(false);
    green_.Set(false);
    yellow_.Set(false);
    buzzer_.Set(false);
}
