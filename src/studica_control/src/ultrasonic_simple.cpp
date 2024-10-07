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

                        VMXChannelIndex ping_channel_index = 10;        // Digital Output, pulsed for 10uS to trigger ranging
                        VMXChannelIndex echo_channel_index = 11;                // "Two Pulse" (timing of rising and falling edge) Timer Input

                        // Suggested measurement cycle period is 60ms

                        uint32_t microseconds_per_inch = 148;

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

                        InputCaptureConfig inputcap_config;

                        /* For "Period of a Pulse" Input Capture, the duration of a pulse occuring on single timer input pin */
                        /* is measured using one of the VMX-pi timer circuits.                                               */
                        /* The "Slave Mode Reset" timer mode is used.  When a rising edge occurs, the counter is reset to 0. */
                        /* Then, when the falling edge occurs, the counter value is captured into the second capture channel.*/

                        inputcap_config.SetSlaveMode(InputCaptureConfig::SLAVEMODE_RESET);

                        /* The source of the Rising Edge Input is used as the Trigger Input Source, and because it's dynamic,*/
                        /* when Resource Activation occurs later, the source is set to the VMXChannel connected to the timer.*/
                        inputcap_config.SetSlaveModeTriggerSource(InputCaptureConfig::TRIGGER_DYNAMIC); 

                        /* The dynamically-selected VMXChannel will also be routed to each of the two timer channels.        */
                        inputcap_config.SetCaptureChannelSource(InputCaptureConfig::CH1, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);
                        inputcap_config.SetCaptureChannelSource(InputCaptureConfig::CH2, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);

                        /* Due to the STM32 Timer Trigger routing limitations, which Timer Capture Channel is assigned the   */
                        /* Rising (active) Edge changes depending upon which physical input pin is driving the timer.        */

                        if (vmx.io.ChannelSupportsCapability(echo_channel_index, VMXChannelCapability::InputCaptureInput2)) {
                                // Second Timer Input:  Second Timer Channel must handle the rising edge
                                inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_FALLING);
                                inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_RISING);
                        } else {
                                // First Timer Input:  First Timer Channel must handle the rising edge
                                inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_RISING);
                                inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_FALLING);
                        }

                        /* Capture occurs on each and every input signal edge transition. */
                        inputcap_config.SetCaptureChannelPrescaler(InputCaptureConfig::CH1, InputCaptureConfig::x1);
                        inputcap_config.SetCaptureChannelPrescaler(InputCaptureConfig::CH2, InputCaptureConfig::x1);

                        /* Digitally filter 2 successive samples when looking for edges  */
                        uint8_t filter_number = inputcap_config.GetClosestCaptureCaptureFilterNumSamples(2);
                        inputcap_config.SetCaptureChannelFilter(InputCaptureConfig::CH1, filter_number);
                        inputcap_config.SetCaptureChannelFilter(InputCaptureConfig::CH2, filter_number);

                        VMXChannelInfo echo_chaninfo(echo_channel_index, VMXChannelCapability::InputCaptureInput2);

                        /* If the input is no longer present, do not automatically clear the Capture Channel Count */
                        inputcap_config.SetStallAction(InputCaptureConfig::ACTION_NONE);

                        if (!vmx.io.ActivateSinglechannelResource(echo_chaninfo, &inputcap_config, echo_inputcap_res_handle, &vmxerr)) {
                                printf("Error Activating Singlechannel Resource InputCapture for Channel index %d.\n", echo_channel_index);
                                DisplayVMXError(vmxerr);
                        } else {
                                printf("Echo (Input Capture) Channel %d activated on Resource type %d, index %d\n", echo_channel_index,
                                                EXTRACT_VMX_RESOURCE_TYPE(echo_inputcap_res_handle),
                                                EXTRACT_VMX_RESOURCE_INDEX(echo_inputcap_res_handle));
                        }

                        for (int i = 0; i < 100; i++) {
                                if (!vmx.io.DIO_Pulse(ping_output_res_handle, true /*high*/, 10, &vmxerr)) {
                                        printf("Error triggering Pulse.\n");
                                        DisplayVMXError(vmxerr);
                                }
                                vmx.time.DelayMilliseconds(5);
                                uint32_t ch1_count = 0;
                                uint32_t ch2_count = 0;

                                if (!vmx.io.InputCapture_GetChannelCounts(echo_inputcap_res_handle, ch1_count, ch2_count, &vmxerr)) {
                                        printf("Error retrieving Input Capture Count.\n");
                                        DisplayVMXError(vmxerr);
                                } else {
                                        printf("Ch1 Count:  %d, Ch2 Count:  %d, Distance (inches):  %d\n", ch1_count, ch2_count, ch2_count / microseconds_per_inch);
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