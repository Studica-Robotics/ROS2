/*
 * ultrasonic.cpp
 *
 * Example demonstrating reading distance from a HC-SR04 Ultrasonic Sensor.
 *
 *  Created on: 13 Aug 2016
 *      Author: pi
 */

#include <stdio.h>  /* printf() */
#include <string.h> /* memcpy() */
#include <inttypes.h>
#include "VMXPi.h"

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

void DisplayVMXError(VMXErrorCode vmxerr) {
        const char *p_err_description = GetVMXErrorString(vmxerr);
        printf("VMXError %d:  %s\n", vmxerr, p_err_description);
}

static constexpr double kPingTime = 10 * 1e-6;

int main(int argc, char *argv[])
{
        bool realtime = true;
        uint8_t update_rate_hz = 50;
        VMXPi vmx(realtime, update_rate_hz);
        try {
                if(vmx.IsOpen()) {

                        VMXChannelIndex ping_channel_index = 8;         // Digital Output, pulsed for 10uS to trigger ranging
                        VMXChannelIndex echo_channel_index = 9;         // "Two Pulse" (timing of rising and falling edge) Timer Input

                        // Suggested measurement cycle period is 60ms

                        // uint32_t microseconds_per_inch = 148;

                        VMXResourceHandle ping_output_res_handle;
                        VMXResourceHandle echo_inputcap_res_handle;

                        VMXErrorCode vmxerr;

                        double pulseLength = kPingTime;
                        uint32_t num_microseconds = static_cast<uint32_t>(pulseLength / 1.0e-6);

                        printf("Trigger Num Microseconds:  %d\n", num_microseconds);

                        DIOConfig dio_config(DIOConfig::OutputMode::PUSHPULL);
                        if (!vmx.io.ActivateSinglechannelResource(VMXChannelInfo(ping_channel_index, VMXChannelCapability::DigitalOutput), &dio_config, 
                                        ping_output_res_handle, &vmxerr)) {
                                printf("Error Activating Singlechannel Resource DIO for Channel index %d.\n", ping_channel_index);
                                DisplayVMXError(vmxerr);
                        } else {
                                printf("Ping (Digital Output) Channel %d activated on Resource type %d, index %d\n", ping_channel_index,
                                                EXTRACT_VMX_RESOURCE_TYPE(ping_output_res_handle),
                                                EXTRACT_VMX_RESOURCE_INDEX(ping_output_res_handle));
                        }

                        PWMCaptureConfig inputcap_config;

                        inputcap_config.SetStallAction(InputCaptureConfig::ACTION_NONE);  // Always leave the previous values even if signal is no longer present

                        VMXChannelInfo echo_chaninfo(echo_channel_index, VMXChannelCapability::PWMCaptureInput2);

                        if (!vmx.io.ActivateSinglechannelResource(echo_chaninfo, &inputcap_config, echo_inputcap_res_handle, &vmxerr)) {
                                printf("Error Activating Singlechannel Resource InputCapture for Channel index %d.\n", echo_channel_index);
                                DisplayVMXError(vmxerr);
                        } else {
                                printf("Echo (Input Capture) Channel %d activated on Resource type %d, index %d\n", echo_channel_index,
                                                EXTRACT_VMX_RESOURCE_TYPE(echo_inputcap_res_handle),
                                                EXTRACT_VMX_RESOURCE_INDEX(echo_inputcap_res_handle));
                        }

                        for (int i = 0; i < 100; i++) {
                                /*
                                if (!vmx.io.InputCapture_Reset(echo_inputcap_res_handle, &vmxerr)) {
                                        printf("Error resetting echo pulse timer.\n");
                                        DisplayVMXError(vmxerr);
                                }
                                */
                                if (!vmx.io.DIO_Pulse(ping_output_res_handle, true /*high*/, 10, &vmxerr)) {
                                        printf("Error triggering Pulse.\n");
                                        DisplayVMXError(vmxerr);
                                }
                                vmx.time.DelayMilliseconds(5);
                                //int32_t count = 0;
                                uint32_t ch1_count;
                                uint32_t ch2_count;
                                if (!vmx.io.InputCapture_GetChannelCounts(echo_inputcap_res_handle, ch1_count, ch2_count, &vmxerr)) {
                                        printf("Error retrieving PWM Capture Count.\n");
                                        DisplayVMXError(vmxerr);
                                } else {
                                        printf("Count1:  %d, Count2:  %d, Distance (inches):  %d\n", ch1_count, ch2_count, (ch2_count * inputcap_config.GetMicrosecondsPerTick()) / microseconds_per_inch);
                                        bool forward_direction = false;
                                        bool active = false;
                                        if (vmx.io.InputCapture_InputStatus(echo_inputcap_res_handle, forward_direction, active, &vmxerr)) {
                                                printf("Direction:  %s, Active:  %s\n", (forward_direction ? "FORWARD" : "REVERSE"), (active ? "YES" : "NO"));
                                        }
                                }
                                vmx.time.DelayMilliseconds(55);
                        }

                        printf("Ultrasonic (HC-SR04) sensor test completed.  Shutting down now.\n");
                } else {
                        printf("Error:  Unable to open VMX Client.\n");
                        printf("\n");
                        printf("        - Is pigpio (or the system resources it requires) in use by another process?\n");
                        printf("        - Does this application have root privileges?\n");
                }
        }
        catch(const std::exception& ex){
                printf("Caught exception:  %s", ex.what());
        }
}

