#ifndef VMX_MANAGER_H
#define VMX_MANAGER_H

#include <memory>
#include "VMXPi.h"

namespace studica_driver
{

class VMXManager {
public:
    static VMXManager& getInstance();

    VMXManager(const VMXManager&) = delete;
    VMXManager& operator=(const VMXManager&) = delete;

    void initialize();

    std::shared_ptr<VMXPi> getVMX();

    // Pin management
    void setPinUsed(int pin);
    void setPinUnused(int pin);
    bool isPinUsed(int pin);

private:
    std::shared_ptr<VMXPi> vmx_;
    bool used_pins[22] = {false};

    VMXManager();
    ~VMXManager();
};

} // namespace studica_driver

#endif // VMX_MANAGER_H
