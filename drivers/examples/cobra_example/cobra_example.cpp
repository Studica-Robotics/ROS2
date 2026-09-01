#include "VMXPi.h"
#include "cobra.hpp"
#include <chrono>
#include <thread>

int main(int argc, char* argv[])
{

    std::shared_ptr<VMXPi> vmx_ = std::make_shared<VMXPi>(true, 50);

    studica_driver::Cobra cobra(vmx_, 5.0F);
    for (uint8_t ch = 0; ch < 4; ch++)
    {
        float voltage = cobra.GetVoltage(ch);
        int value = cobra.GetRawValue(ch);
        printf("Channel %d — Voltage: %f V, ADC Value: %d\n", ch, voltage, value);
    }
}