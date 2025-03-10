#include "cobra.h"
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    studica_driver::Cobra cobra(5.0F);
    float voltage = cobra.GetVoltage(1);
    int value = cobra.GetRawValue(1);
    printf("\nVoltage: %f", voltage);
    printf("\nADC Value: %d", value);
}