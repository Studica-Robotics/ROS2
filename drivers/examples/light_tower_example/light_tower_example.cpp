#include "VMXPi.h"
#include "light_tower.hpp"
#include <chrono>
#include <cstdio>
#include <thread>

static void pause(int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

int main(int argc, char* argv[])
{
    std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);

    // Pins: continuous=0, red=1, green=2, yellow=3, buzzer=4
    studica_driver::LightTower tower(0, 1, 2, 3, 4, vmx);

    printf("Red solid\n");
    tower.SetContinuous(true);
    tower.SetRed(true);
    pause(2);

    printf("Green solid\n");
    tower.AllOff();
    tower.SetContinuous(true);
    tower.SetGreen(true);
    pause(2);

    printf("Yellow solid\n");
    tower.AllOff();
    tower.SetContinuous(true);
    tower.SetYellow(true);
    pause(2);

    printf("Buzzer\n");
    tower.AllOff();
    tower.SetContinuous(true);
    tower.SetBuzzer(true);
    pause(2);

    // Hardware blink: continuous LOW, color HIGH — flash rate driven by the tower hardware
    printf("Red hardware blink\n");
    tower.AllOff();
    tower.SetContinuous(false);
    tower.SetRed(true);
    pause(4);

    printf("All off\n");
    tower.AllOff();

    return 0;
}
