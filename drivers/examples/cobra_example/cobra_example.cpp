#include "cobra.h"
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    float vref = 0.5;
    int muxch = 1;
    studica_driver::Cobra cobra(vref, muxch);
    for (int i = 0; i < 10; ++i) {
        printf("%d |  Raw ADC Value: %d, ", i, cobra.GetRawValue());
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        printf("Voltage: %f\n", cobra.GetVoltage());
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }
    return 0;
}