#include "VMXManager.h"
using namespace studica_driver;

VMXManager& VMXManager::getInstance() {
    static VMXManager instance; // Only created once, then reused
    return instance;
}

VMXManager::VMXManager() {
    initialize();
}

VMXManager::~VMXManager() {
    // Clean-up code (if necessary)
}

void VMXManager::initialize() {
    vmx_ = std::make_shared<VMXPi>(true, 50); // Initialize VMX HAL with appropriate arguments
}

std::shared_ptr<VMXPi> VMXManager::getVMX() {
    return vmx_;
}

void VMXManager::setPinUsed(int pin) {
    if (pin >= 0 && pin < 22) {
        used_pins[pin] = true;
    }
}

void VMXManager::setPinUnused(int pin) {
    if (pin >= 0 && pin < 22) {
        used_pins[pin] = false;
    }
}

bool VMXManager::isPinUsed(int pin) {
    if (pin >= 0 && pin < 22) {
        return used_pins[pin];
    }
    return false; // Out of bounds pin
}
