#pragma once

#include "VMXPi.h"
#include "analog_input.h"
#include <cmath>
#include <stdio.h>

namespace studica_driver
{

    class Sharp
    {
        public:
            Sharp(VMXChannelIndex port, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
            ~Sharp();

            float GetDistance();
            float GetVoltage();

        private:
            std::shared_ptr<VMXPi> vmx_;
            std::shared_ptr<AnalogInput> analog_input_;
            VMXChannelIndex port_;
    };

} // namespace studica_driver
