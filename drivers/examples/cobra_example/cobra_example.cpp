#include "cobra.h"
#include <thread>
#include <chrono>
#include "VMXPi.h"

int main(int argc, char *argv[]) {

    std::shared_ptr<VMXPi> vmx_ = std::make_shared<VMXPi>(true, 50);

    studica_driver::Cobra cobra(vmx_, 5.0F);
    float voltage = cobra.GetVoltage(1);
    int value = cobra.GetRawValue(1);
    printf("\nVoltage: %f", voltage);
    printf("\nADC Value: %d", value);
}