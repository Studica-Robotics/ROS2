#############################################################################################
#
#   Teddy Library for Python
#
#   Created by: James Taylor
#   HAL By: Scott Libert
#
#   Date: 2024/02/27
#
#   All print statements are intended for logging purposes and not realtime user prints
#
#   Todo:
#       - add more stuff
#
#############################################################################################

# imports
import imp
import sys
from enum import IntEnum

# Valid ports to be used by USER
class Port(IntEnum):
    SERVO0 = 12
    SERVO1 = 13
    SERVO2 = 14
    SERVO3 = 15
    SERVO4 = 16
    SERVO5 = 17
    HCDO0 = 18
    HCDO1 = 19
    HCDO2 = 20
    HCDO3 = 21
    FLEXDIO0 = 8
    FLEXDIO1 = 9
    FLEXDIO2 = 10
    FLEXDIO3 = 11
    AI0 = 22
    AI1 = 23
    AI2 = 24
    AI3 = 25
    M0_A = 0
    M0_B = 1
    M1_A = 2
    M1_B = 3
    M2_A = 4
    M2_B = 5
    M3_A = 6
    M3_B = 7

# Hardware Map used for debug statements
class _ZeroMap:
    zeromap = ["M0_A", "M0_B",
                "M1_A", "M1_B",
                "M2_A", "M2_B",
                "M3_A", "M3_B",
                "FLEXDIO0", "FLEXDIO1",
                "FLEXDIO2", "FLEXDIO3",
                "SERVO0", "SERVO1",
                "SERVO2", "SERVO3",
                "SERVO4", "SERVO5",
                "HCDO0", "HCDO1",
                "HCDO2", "HCDO3",
                "AI0", "AI1",
                "AI2", "AI3"]

# Main Class
class Teddy:
    __StartButton = 0 
    __StopButton = 0
    __StartLED = 0
    __StopLED = 0
    __WD = 0
    
    def __init__(self):
        # Grab Zero HAL
        global zero
        global vmxzero
        sys.path.append('/usr/local/lib/vmxpi/')
        vmxzero = imp.load_source('vmxpi_hal_python', '/usr/local/lib/vmxpi/vmxpi_hal_python.py')
        zero = vmxzero.VMXPi(True, 50)
        if zero.IsOpen():
            print("Zero: Library Intialized")
        else:
            print("Zero: Error Initializing Library")
    
    # System Info
    def GetSystemInfo(self):
        print("Zero: VMX Zero")
        print("Zero: HAL Version: ", zero.getVersion().GetHALVersion())
        print("Zero: Firmware Version: ", zero.getVersion().GetFirmwareVersion())
        #print("Hardware Version: ", zero.getVersion().GetHardwareVersion()) Needs implementing?
        #print("Serial Number: ", zero.getVerison().GetSerialNumber()) Needs implementing?

    # Battery Info
    def GetBatteryVoltage(self):
        return zero.getPower().GetSystemVoltage()

    # Delay Functions
    def DelayMs(self, ms):
        zero.getTime().DelayMilliseconds(ms)
    
    def DelayS(self, s):
        zero.getTime().DelaySeconds(s)
    
    def DelayUs(self, us):
        zero.getTime().DelayMicroseconds(us)
        
    def __InitBackgroundStuff(self):
        
        self.__StartButton = DigitalInput(8)
        self.__StopButton = DigitalInput(9)
        self.__StartLED = DigitalOutput(20)
        self.__StopLED = DigitalOutput(21)
        self.__WD = _Watchdog() 

    ###################################
    #         Event Loops             #
    ###################################

    # Runs once and goes to the Init Loop
    def Init(self):
        print("Zero: Init Method must be overriden!")

    # Runs until the green button on Teddy is pushed
    # Update rate 50Hz - 200Hz
    def Init_Loop(self):
        print("Zero: Init_Loop Method must be overriden!")

    # Runs once after the green button is pushed then goes to Run Loop
    def Run_Init(self):
        print("Zero: Run_Init Method must be overriden!")

    # Runs until the stop button is hit or an error
    # Update Rate 50Hz - 200Hz
    def Run_Loop(self):
        print("Zero: Run_Loop Method must be overriden!")
    
    # Runs once after the stop button is hit and goes to Init unless there is an error
    def Stop(self):
        print("Zero: Stop Method must be overriden!")
        
    ###################################
    #         Event Scheduler         #
    ###################################
    def Start(self):
        print("Zero: Starting Program Code")
        self.__InitBackgroundStuff()
        while(True):
            print("Zero: Program Starting!")
            while(self.__StartButton.Get() == 1):
                self.__StartLED.Set(True)
                self.__StopLED.Set(True)
            while(self.__StartButton.Get() == 0):
                pass
            print("Zero: Init Method!")
            self.Init()
            print("Zero: Init Loop Method!")
            while(self.__StartButton.Get() == 1):
                start_time = zero.getTime().GetCurrentMicroseconds()
                self.__StartLED.Set(True)
                self.__StopLED.Set(False)
                self.Init_Loop()
                end_time = zero.getTime().GetCurrentMicroseconds()
                delay = 20000 - (end_time - start_time)
                if (delay < 0):
                    print("Zero: Init Loop Overrun Time: {}ms".format(20 + (delay*-1)/1000))
                else:
                    self.DelayUs(delay)
            print("Zero: Run Init Method!")
            self.Run_Init()
            print("Zero: Run Loop Method!")
            while(self.__StopButton.Get() == 1):
                start_time = zero.getTime().GetCurrentMicroseconds()
                self.__StartLED.Set(False)
                self.__StopLED.Set(True)
                self.__WD.Feed()
                self.Run_Loop()
                end_time = zero.getTime().GetCurrentMicroseconds()
                delay = 20000 - (end_time - start_time)
                if (delay < 0):
                    print("Zero: Run Loop Overrun Time: {}ms".format(20 + (delay*-1)/1000))
                else:
                    self.DelayUs(delay)
            print("Zero: Stop Method!")
            self.__WD.Expire()
            self.Stop()

# Servo Class    
class Servo:

    #Local Variables
    __local_port = 0
    __prev_pwm_servo_value = 0
    __pwm_res_handle = 0
    __min = 0
    __max = 0
    __min_us = 0
    __max_us = 0
    __center_us = 0

    # Servo Constructor
    #
    # @param port - to assign the servo pwm      #Required
    # @param min - angle or speed for the servo   #Optional
    # @param max - angle or speed for the servo   #Optional
    def __init__(self, port, min = -150, max = 150):
        if port in range (8, 22, 1):
            pwmgen_cfg = vmxzero.PWMGeneratorConfig(50) # Servos are 50Hz
            pwmgen_cfg.SetMaxDutyCycleValue(5000) # Increase the servo bandwidth to allow better accuracy
            success, self.__pwm_res_handle, vmxerr = zero.getIO().ActivateSinglechannelResource(vmxzero.VMXChannelInfo(port, vmxzero.PWMGeneratorOutput), pwmgen_cfg)
            self.__local_port = port
            self.__prev_pwm_servo_value = min - 1
            self.__min = min
            self.__max = max
            self.SetBounds(0.5, 1.5, 2.5)
            if not success:
                print("Zero: Failed to initialize servo for port {}".format(_ZeroMap.zeromap[port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else: 
                print("Zero: Successfully initialized port {} as a servo output".format(_ZeroMap.zeromap[port]))
        else:
            print("Zero: Port {} is not a valid servo port!".format(_ZeroMap.zeromap[port]))

    # Convert us to int to be used by the map function
    # This will keep the servo into bounds set by the user
    def SetBounds(self, min = 0.5, center = 1.5, max = 2.5):
        min = int((min / 20) * 5000)
        center = int((center / 20) * 5000)
        max = int((max / 20) * 5000)
        self.__min_us = min
        self.__max_us = max
        self.__center_us = center

    # Map function for servos
    #
    # Servos have a range of 0.5ms to 2.5ms duty cycle at 50Hz
    # 2.5% (5000) = 0.5ms ~ 125 min
    # 7.5% (5000) = 1.5ms ~ 375 center
    # 12.5% (5000) = 2.5ms ~ 625 max
    def __Map(self, value):
        if (value < self.__min):
            value = self.__min
        elif (value > self.__max):
            value = self.__max
        return int((value - (self.__min)) * (self.__max_us - self.__min_us) / (self.__max - (self.__min)) + self.__min_us)
        
    # Standard Servo
    def SetAngle(self, angle):
        if(self.__prev_pwm_servo_value != angle):
            success, vmxerr = zero.getIO().PWMGenerator_SetDutyCycle(self.__pwm_res_handle, 0, self.__Map(angle))
            self.__prev_pwm_servo_value = angle
            if not success:
                print("Zero: Failed to set duty cycle for servo on port {}".format(_ZeroMap.zeromap[self.__local_port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else:
                print("Zero: PWM Duty cycle set on port {} at {}".format(_ZeroMap.zeromap[self.__local_port], angle))

    # Continuous Servo
    def SetSpeed(self, speed):
        if(self.__prev_pwm_servo_value != speed):
            success, vmxerr = zero.getIO().PWMGenerator_SetDutyCycle(self.__pwm_res_handle, 0, self.__Map(speed))
            self.__prev_pwm_servo_value = speed
            if not success:
                print("Zero: Failed to set duty cycle for servo port {}".format(_ZeroMap.zeromap[self.__local_port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else:
                print("Zero: PWM Duty cycle set on port {} at {}".format(_ZeroMap.zeromap[self.__local_port], speed))

# Digital Input Class
class DigitalInput:
    
    __local_port = 0
    __di_res_handle = 0

    def __init__(self, port):
        if port in range (8, 12, 1):
            dio_config = vmxzero.DIOConfig()
            success, self.__di_res_handle, vmxerr = zero.getIO().ActivateSinglechannelResource(vmxzero.VMXChannelInfo(port, vmxzero.DigitalInput), dio_config)
            self.__local_port = port
            if not success:
                print("Zero: Failed to initialize port {} as a digital input".format(_ZeroMap.zeromap[port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else:
                print("Zero: Successfully initialized port {} as a digital input".format(_ZeroMap.zeromap[port]))
        else:
            print("Zero: Port {} is not a valid digital input port!".format(_ZeroMap.zeromap[port]))

    # Digital Get 
    # Returns 1 or 0 and -1 (error)
    # All inputs are normally pulled high
    def Get(self):
        success, high, vmxerr = zero.getIO().DIO_Get(self.__di_res_handle)
        if not success:
            print("Zero: Failed to read digital input on port {}".format(_ZeroMap.zeromap[self.__local_port]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            return -1
        else:
            return (1 if (high) else 0)

# Digital Ouptput Class
class DigitalOutput:
    
    __local_port = 0
    __do_res_handle = 0
    __prev_value = False

    def __init__(self, port):
        if port in range (8, 22, 1):
            dio_config = vmxzero.DIOConfig()
            success, self.__do_res_handle, vmxerr = zero.getIO().ActivateSinglechannelResource(vmxzero.VMXChannelInfo(port, vmxzero.DigitalOutput), dio_config)
            self.__local_port = port
            if not success:
                print("Zero: Failed to initialize port {} as a digital output".format(_ZeroMap.zeromap[port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else:
                print("Zero: Successfully initialized port {} as a digital output".format(_ZeroMap.zeromap[port]))
        else:
            print("Zero: Port {} is not a valid digital output port!".format(_ZeroMap.zeromap[port]))

    # Digital Set 
    # 
    # @param value - True or False
    def Set(self, value):
        if (self.__prev_value != value):
            success, vmxerr = zero.getIO().DIO_Set(self.__do_res_handle, value)
            if not success:
                print("Zero: Failed to set digital output on port {}".format(_ZeroMap.zeromap[self.__local_port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else: 
                self.__prev_value = value

# Blink Class (Blinks a digital output at a set rate)
class Blink:
    
    #Local Variables
    __local_port = 0
    __prev_pwm_value = 0
    __pwm_res_handle = 0
    __min = 0
    __max = 0
    
    def __init__(self, port, period):
        if port in range (8, 22, 1):
            pwmgen_cfg = vmxzero.PWMGeneratorConfig(period) # Hz
            pwmgen_cfg.SetMaxDutyCycleValue(5000) # Increase the bandwidth to allow better accuracy
            success, self.__pwm_res_handle, vmxerr = zero.getIO().ActivateSinglechannelResource(vmxzero.VMXChannelInfo(port, vmxzero.PWMGeneratorOutput), pwmgen_cfg)
            self.__local_port = port
            self.__prev_pwm_value = -1
            self.__min = 0
            self.__max = 100
            if not success:
                print("Zero: Failed to initialize blink for port {}".format(_ZeroMap.zeromap[port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else: 
                print("Zero: Successfully initialized port {} as a blink output".format(_ZeroMap.zeromap[port]))
        else:
            print("Zero: Port {} is not a valid blink port!".format(_ZeroMap.zeromap[port]))
            
    def __Map(self, value):
        if (value < self.__min):
            value = self.__min
        elif (value > self.__max):
            value = self.__max
        return int((value - (self.__min)) * (5000 - 0) / (self.__max - (self.__min)) + 0)
    
    def SetRate(self, rate):
        if(self.__prev_pwm_value != rate):
            success, vmxerr = zero.getIO().PWMGenerator_SetDutyCycle(self.__pwm_res_handle, 0, self.__Map(rate))
            self.__prev_pwm_value = rate
            if not success:
                print("Zero: Failed to set duty cycle for blink on port {}".format(_ZeroMap.zeromap[self.__local_port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else:
                print("Zero: PWM Duty cycle set on port {} at {}".format(_ZeroMap.zeromap[self.__local_port], rate))

# Analog Input Class
class AnalogInput:

    #Local Variables
    __local_port = 0
    __ai_res_handle = 0

    def __init__(self, port):
        if port in range (22, 26, 1):
            accum_config = vmxzero.AccumulatorConfig()
            success, self.__ai_res_handle, vmxerr = zero.getIO().ActivateSinglechannelResource(vmxzero.VMXChannelInfo(port, vmxzero.AccumulatorInput), accum_config)
            self.__local_port = port
            if not success:
                print("Zero: Failed to initialize port {} as an analog input".format(_ZeroMap.zeromap[port]))
                print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            else:
                print("Zero: Successfully initialized port {} as an analog input".format(_ZeroMap.zeromap[port]))
        else:
            print("Zero: {} is not a valid analog port!".format(_ZeroMap.zeromap[port]))
    
    def GetVoltage(self):
        success, voltage, vmxerr = zero.getIO().Accumulator_GetAverageVoltage(self.__ai_res_handle)
        if not success:
            print("Zero: Failed to read voltage on port {}".format(_ZeroMap.zeromap[self.__local_port]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            return -1
        else:
            return voltage

    def GetRawValue(self):
        success, value, vmxerr = zero.getIO().Accumulator_GetAverageValue(self.__ai_res_handle)
        if not success:
            print("Zero: Failed to read raw value on port {}".format(_ZeroMap.zeromap[self.__local_port]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            return -1
        else:
            return value

# Sharp Sensor Class, extends the Analog Input class
class Sharp(AnalogInput):

    def __init__(self, port):
        super().__init__(port)
        print("Zero: {} established as a Sharp Input".format(_ZeroMap.zeromap[port]))

    # Returns distance in CM and -1 for out of bounds
    # Range is 10 cm -> 80 cm
    def GetDistanceCM(self):
        distance = ((pow(self.GetVoltage(), -1.2045)) * 27.726)
        if distance < 10 or distance > 80:
            return -1
        else:
            return distance

# Ultrasonic class for 4 pin ultrasonic sensors
class Ultrasonic:

    #Local Variables
    __local_pulse_port = 0
    __local_echo_port = 0
    __pulse_res_handle = 0
    __echo_res_handle = 0

    def __init__(self, pulse_port, echo_port):
        if pulse_port in range (8, 22, 1):
            if echo_port in range (9, 12, 2):
                # Configure Pulse
                pulse_config = vmxzero.DIOConfig(vmxzero.DIOConfig().PUSHPULL)
                success, self.__pulse_res_handle, vmxerr = zero.getIO().ActivateSinglechannelResource(vmxzero.VMXChannelInfo(pulse_port, vmxzero.DigitalOutput), pulse_config)
                self.__local_pulse_port = pulse_port
                if not success:
                    print("Zero: Failed to initialize port {} as a digital output".format(_ZeroMap.zeromap[pulse_port]))
                    print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
                else:
                    print("Zero: Successfully initialized port {} as a digital output".format(_ZeroMap.zeromap[pulse_port]))
                    print("Zero: Port {} set as ultrasonic pulse".format(_ZeroMap.zeromap[pulse_port]))

                # Configure echo
                echo_config = vmxzero.InputCaptureConfig()
                # Slave Mode Reset is used 
                echo_config.SetSlaveMode(vmxzero.InputCaptureConfig().SLAVEMODE_RESET)
                # Rising Edge input is dynamic
                echo_config.SetSlaveModeTriggerSource(vmxzero.InputCaptureConfig().TRIGGER_DYNAMIC)
                # The dynamically-selected VMXChannel should also be routed to both timer channels
                echo_config.SetCaptureChannelSource(vmxzero.InputCaptureConfig().CH1, vmxzero.InputCaptureConfig().CAPTURE_SIGNAL_DYNAMIC)
                echo_config.SetCaptureChannelSource(vmxzero.InputCaptureConfig().CH2, vmxzero.InputCaptureConfig().CAPTURE_SIGNAL_DYNAMIC)
                # Check to see which physical pin is driving the timer
                if (zero.getIO().ChannelSupportsCapability(echo_port, vmxzero.InputCaptureInput2)):
                    # Second Timer Input: Second Timer channel must handle the rising edge
                    echo_config.SetCaptureChannelActiveEdge(vmxzero.InputCaptureConfig().CH1, vmxzero.InputCaptureConfig().ACTIVE_FALLING)
                    echo_config.SetCaptureChannelActiveEdge(vmxzero.InputCaptureConfig().CH2, vmxzero.InputCaptureConfig().ACTIVE_RISING)
                else:
                    # First Timer Input: First Timer Channel must handle the rising edge
                    echo_config.SetCaptureChannelActiveEdge(vmxzero.InputCaptureConfig().CH1, vmxzero.InputCaptureConfig().ACTIVE_RISING)
                    echo_config.SetCaptureChannelActiveEdge(vmxzero.InputCaptureConfig().CH2, vmxzero.InputCaptureConfig().ACTIVE_FALLING)
                # Capture should occur on each and every input signal edge transition
                echo_config.SetCaptureChannelPrescaler(vmxzero.InputCaptureConfig().CH1, vmxzero.InputCaptureConfig().x1)
                echo_config.SetCaptureChannelPrescaler(vmxzero.InputCaptureConfig().CH2, vmxzero.InputCaptureConfig().x1)
                # Digitally filter 2 successive samples when looking for edges
                filter_number = echo_config.GetClosestCaptureCaptureFilterNumSamples(2)
                echo_config.SetCaptureChannelFilter(vmxzero.InputCaptureConfig().CH1, filter_number)
                echo_config.SetCaptureChannelFilter(vmxzero.InputCaptureConfig().CH2, filter_number)
                # If the input is no longer present, do not automatically clear the Capture Channel Count
                echo_config.SetStallAction(vmxzero.InputCaptureConfig().ACTION_NONE)

                success, self.__echo_res_handle, vmxerr = zero.getIO().ActivateSinglechannelResource(vmxzero.VMXChannelInfo(echo_port, vmxzero.InputCaptureInput2), echo_config)
                self.__local_echo_port = echo_port
                if not success:
                    print("Zero: Failed to initialize port {} as a digital input".format(_ZeroMap.zeromap[echo_port]))
                    print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
                else:
                    print("Zero: Successfully initialized port {} as a digital input".format(_ZeroMap.zeromap[echo_port]))
                    print("Zero: Port {} set as ultrasonic echo".format(_ZeroMap.zeromap[echo_port]))
            else:
                print("Zero: {} is not a valid echo port!".format(_ZeroMap.zeromap[echo_port]))
        else:
            print("Zero: {} is not a valid pulse port!".format(_ZeroMap.zeromap[pulse_port]))
            
    # This user needs to call this when a ping needs to be sent out
    def Ping(self):
        success, vmxerr = zero.getIO().DIO_Pulse(self.__pulse_res_handle, True, 10)
        if not success:
            print("Zero: Failed to set pulse on port {}".format(_ZeroMap.zeromap[self.__local_pulse_port]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        zero.getTime().DelayMilliseconds(5)
    
    # After sending the ping the user can call this to get the distance in inches
    def GetDistanceIN(self):
        success, ch1_count, ch2_count, vmxerr = zero.getIO().InputCapture_GetChannelCounts(self.__echo_res_handle)
        if not success:
            print("Zero: Failed to get echo count on port {}".format(_ZeroMap.zeromap[self.__local_echo_port]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            return ch2_count / 148
    
    # After sending the ping the user can call this to get the distance in milimeters 
    def GetDistanceMM(self):
        success, ch1_count, ch2_count, vmxerr = zero.getIO().InputCapture_GetChannelCounts(self.__echo_res_handle)
        if not success:
            print("Zero: Failed to get echo count on port {}".format(_ZeroMap.zeromap[self.__local_echo_port]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            return ch2_count / 58

# Built in NavX Imu class
class IMU:
    
    def __init__(self):
        pass

    def GetPitch(self):
        return zero.getAHRS().GetPitch()

    def GetYaw(self):
        return zero.getAHRS().GetYaw()

    def GetRoll(self):
        return zero.getAHRS().GetRoll()

    def GetCompassHeading(self):
        return zero.getAHRS().GetCompassHeading()

    def ZeroYaw(self):
        zero.getAHRS().ZeroYaw()

    def IsCalibrating(self):
        return zero.getAHRS().IsCalibrating()

    def IsConnected(self):
        return zero.getAHRS().IsConnected()

    def GetByteCount(self):
        return zero.getAHRS().GetByteCount()

    def GetUpdateCount(self):
        return zero.getAHRS().GetUpdateCount()

    def GetLastSensorTimestamp(self):
        return zero.getAHRS().GetLastSensorTimestamp()

    def GetWorldLinearAccelX(self):
        return zero.getAHRS().GetWorldLinearAccelX()

    def GetWorldLinearAccelY(self):
        return zero.getAHRS().GetWorldLinearAccelY()

    def GetWorldLinearAccelZ(self):
        return zero.getAHRS().GetWorldLinearAccelZ()

    def IsMoving(self):
        return zero.getAHRS().IsMoving()

    def IsRotating(self):
        return zero.getAHRS().IsRotating()

    def GetBarometricPressure(self):
        return zero.getAHRS().GetBarometricPressure()

    def GetAltitude(self):
        return zero.getAHRS().GetAltitude()

    def IsAltitudeValid(self):
        return zero.getAHRS().IsAltitudeValid()

    def GetFusedHeading(self):
        return zero.getAHRS().GetFusedHeading()

    def IsMagneticDisturbance(self):
        return zero.getAHRS().IsMagneticDisturbance()

    def IsMagnetometerCalibrated(self):
        return zero.getAHRS().IsMagnetometerCalibrated()

    def GetQuaternionW(self):
        return zero.getAHRS().GetQuaternionW()
    
    def GetQuaternionX(self):
        return zero.getAHRS().GetQuaternionX()

    def GetQuaternionY(self):
        return zero.getAHRS().GetQuaternionY()

    def GetQuaternionZ(self):
        return zero.getAHRS().GetQuaternionZ()

    def ResetDisplacement(self):
        zero.getAHRS().ResetDisplacement()

    def UpdateDisplacement(self, accel_x, accel_y, update_rate_hz, is_moving):
        zero.getAHRS().UpdateDisplacement(accel_x, accel_y, update_rate_hz, is_moving)

    def GetVelocityX(self):
        return zero.getAHRS().GetVelocityX()
    
    def GetVelocityY(self):
        return zero.getAHRS().GetVelocityY()
    
    def GetVelocityZ(self):
        return zero.getAHRS().GetVelocityZ()

    def GetDisplacementX(self):
        return zero.getAHRS().GetDisplacementX()
    
    def GetDisplacementY(self):
        return zero.getAHRS().GetDisplacementY()

    def GetDisplacementZ(self):
        return zero.getAHRS().GetDisplacementZ()   

    def GetAngle(self):
        return zero.getAHRS().GetAngle()

    def GetRate(self):
        return zero.getAHRS().GetRate()

    def Reset(self):
        return zero.getAHRS().Reset()

    def GetRawGyroX(self):
        return zero.getAHRS().GetRawGyroX()

    def GetRawGyroY(self):
        return zero.getAHRS().GetRawGyroY()

    def GetRawGyroZ(self):
        return zero.getAHRS().GetRawGyroZ()

    def GetRawAccelX(self):
        return zero.getAHRS().GetRawAccelX()

    def GetRawAccelY(self):
        return zero.getAHRS().GetRawAccelY()

    def GetRawAccelZ(self):
        return zero.getAHRS().GetRawAccelZ()

    def GetRawMagX(self):
        return zero.getAHRS().GetRawMagX()

    def GetRawMagY(self):
        return zero.getAHRS().GetRawMagY()

    def GetRawMagZ(self):
        return zero.getAHRS().GetRawMagZ()

    def GetPressure(self):
        return zero.getAHRS().GetPressure()

    def GetTempC(self):
        return zero.getAHRS().GetTempC()

    def GetBoardYawAxis(self):
        return zero.getAHRS().GetBoardYawAxis()

    def GetFirmwareVersion(self):
        return zero.getAHRS().GetFirmwareVersion()

    def RegisterCallback(self, callback, callback_context):
        zero.getAHRS().RegisterCallback(callback, callback_context)

    def DeregisterCallback(self, callback):
        zero.getAHRS().DeregisterCallback(callback)

    def BlockOnNewCurrentRegisterData(self, timeout_ms, first_reg_addr_out, p_data_out, requested_len, p_len_out):
        zero.getAHRS().BlockOnNewCurrentRegisterData(timeout_ms, first_reg_addr_out, p_data_out, requested_len, p_len_out)
    
    def ReadConfigurationData(self, first_reg_addr, p_data_out, requested_len):
        return zero.getAHRS().ReadConfigurationData(first_reg_addr, p_data_out, requested_len)

    def GetActualUpdateRate(self):
        return zero.getAHRS().GetActualUpdateRate()

    def GetRequestedUpdateRate(self):
        return zero.getAHRS().GetRequestedUpdateRate()

    def Stop(self):
        zero.getAHRS().Stop()

# Cobra class that uses the external i2c ADC
class Cobra:

    __i2c_res_handle = 0
    __device_address = 0x48
    __vRef = 0
    __mode = 0x0000
    __gain = 0x0400
    __rate = 0x0080
    __configOSSingle = 0x8000
    __config = 0x000
    __configMuxSingle0 = 0x4000
    __configMuxSingle1 = 0x5000
    __configMuxSingle2 = 0x6000
    __configMuxSingle3 = 0x7000

    
    def __init__(self, vRef = 5.0):
        i2c_channel_sda = vmxzero.VMXChannelInfo(zero.getIO().GetSoleChannelIndex(vmxzero.I2C_SDA), vmxzero.I2C_SDA)
        i2c_channel_scl = vmxzero.VMXChannelInfo(zero.getIO().GetSoleChannelIndex(vmxzero.I2C_SCL), vmxzero.I2C_SCL)
        i2c_config = vmxzero.I2CConfig()
        success, self.__i2c_res_handle, vmxerr = zero.getIO().ActivateDualchannelResource(i2c_channel_sda, i2c_channel_scl, i2c_config)
        self.__vRef = vRef
        if not success:
            print("Zero: Failed to initialize cobra")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully initialized cobra")
            self.__IsConnected()
    
    def __GetSingle(self, channel):

        if (channel > 3):
            return 0

        self.__config = self.__configOSSingle | self.__mode | self.__rate | self.__gain
        if channel == 0:
                self.__config = self.__config | self.__configMuxSingle0
        elif channel == 1:
                self.__config = self.__config | self.__configMuxSingle1
        elif channel == 2:
                self.__config = self.__config | self.__configMuxSingle2
        elif channel == 3:
                self.__config = self.__config | self.__configMuxSingle3
        
        tx_data = vmxzero.DirectBuffer(3)
        rx_data = vmxzero.DirectBuffer(2)
        tx_data[0] = 0x0001
        tx_data[1] = self.__config >> 8
        tx_data[2] = self.__config & 0xFF
        success, vmxerr = zero.getIO().I2C_Transaction(self.__i2c_res_handle, self.__device_address, tx_data, 3, rx_data, 0)
        #success, vmxerr = zero.getIO().I2C_Write(self.__i2c_res_handle, self.__device_address, 0x0001, tx_data, 2)
        if not success:
            print("Zero: Cobra data not sent")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))

        tx_data2 = vmxzero.DirectBuffer(2)
        tx_data2[0] = 0x0000
        tx_data2[1] = 0x0000
        success, vmxerr = zero.getIO().I2C_Transaction(self.__i2c_res_handle, self.__device_address, tx_data2, 2, rx_data, 2)
        if not success:
            print("Zero: Cobra transaction not completed")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))  
        raw = vmxzero.cdata(rx_data, 2)
        value =  ((ord(raw[0]) << 8) & 0xFF00) + (ord(raw[1]) & 0xFF)
        return value >> 4          

    def __IsConnected(self):
        rx_data = vmxzero.DirectBuffer(1)
        success, vmxerr = zero.getIO().I2C_Read(self.__i2c_res_handle, self.__device_address, 0, rx_data, 1)
        if not success:
            print("Zero: Cobra not connected")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Cobra connected")

    def GetRawValue(self, channel):
        return self.__GetSingle(channel)

    def GetAllRawValues(self):
        values = [0, 0, 0, 0]
        for i in range(4):
            values[i] = self.__GetSingle(i)
        return values

    def GetVoltage(self, channel):
        return (self.__GetSingle(channel))*(self.__vRef/2048)

    def GetAllVoltages(self):
        values = [0, 0, 0, 0]
        for i in range(4):
            values[i] = (self.__GetSingle(i))*(self.__vRef/2048)
        return values

# Encoder Class
class Encoder:
    
    #Local Variables
    __local_port_a = 0
    __local_port_b = 0
    __enc_res_handle = 0
    
    def __init__(self, port_a, port_b):
        enc_channel_a = vmxzero.VMXChannelInfo(port_a, vmxzero.EncoderAInput)
        enc_channel_b = vmxzero.VMXChannelInfo(port_b, vmxzero.EncoderBInput)
        enc_config = vmxzero.EncoderConfig(vmxzero.EncoderConfig.x4)
        success, self.__enc_res_handle, vmxerr = zero.getIO().ActivateDualchannelResource(enc_channel_a, enc_channel_b, enc_config)
        self.__local_port_a = port_a
        self.__local_port_b = port_b
        if not success:
            print("Zero: Failed to initialize port {} and {} as an encoder input".format(_ZeroMap.zeromap[port_a], _ZeroMap.zeromap[port_b]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully initialized port {} and {} as an encoder input".format(_ZeroMap.zeromap[port_a], _ZeroMap.zeromap[port_b]))
    
    def GetCount(self):
        success, count, vmxerr = zero.getIO().Encoder_GetCount(self.__enc_res_handle)
        if not success:
            print("Zero: Error reading encoder count on ports {} & {}".format(_ZeroMap.zeromap[self.__local_port_a], _ZeroMap.zeromap[self.__local_port_b]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        return count
        
    def GetDirection(self):
        success, direction, vmxerr = zero.getIO().Encoder_GetDirection(self.__enc_res_handle)
        if not success:
            print("Zero: Error reading encoder direction on ports {} & {}".format(_ZeroMap.zeromap[self.__local_port_a], _ZeroMap.zeromap[self.__local_port_b]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            return "Forward" if direction == vmxzero.VMXIO.EncoderForward else "Reverse"
        
    def Reset(self):
        success, vmxerr = zero.getIO().Encoder_Reset(self.__enc_res_handle)
        if not success:
            print("Zero: Error resetting encoder on ports {} & {}".format(_ZeroMap.zeromap[self.__local_port_a], _ZeroMap.zeromap[self.__local_port_b]))
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else: 
            print("Zero: Encoder on ports {} & {}, has been reset".format(_ZeroMap.zeromap[self.__local_port_a], _ZeroMap.zeromap[self.__local_port_b]))

# Watchdog class to enable outputs
class _Watchdog:
    
    def __init__(self):
        success, vmxerr = zero.getIO().SetWatchdogManagedOutputs(True, True, True)
        if not success:
            print("Zero: Failed to set watchdog managed outputs")
            print("Zero: {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else: 
            print("Zero: Watchdog managed outputs set")
        
        success, vmxerr = zero.getIO().SetWatchdogTimeoutPeriodMS(250)
        if not success:
            print("Zero: Failed to set watchdog timeout period")
            print("Zero: {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else: 
            print("Zero: Watchdog Timeout Period Set to 250ms")
        
        success, vmxerr = zero.getIO().SetWatchdogEnabled(True)
        if not success:
            print("Zero: Failed to enable watchdog")
            print("Zero: {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Watchdog enabled!")
            
        for i in range(3):
            success, vmxerr = zero.getIO().ExpireWatchdogNow()
            if not success:
                print("Zero: Watchdog does not want to expire, Bad Dog!")
                print("Zero: {}".format(vmxzero.GetVMXErrorString(vmxerr)))
    
    def Feed(self):
        success, vmxerr = zero.getIO().FeedWatchdog()
        if not success:
            print("Zero: Feeding watchdog")
            print("Zero: {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            
    def Expire(self):
        for i in range(3):
            success, vmxerr = zero.getIO().ExpireWatchdogNow()
            if not success:
                print("Zero: Watchdog does not want to expire, Bad Dog!")
                print("Zero: {}".format(vmxzero.GetVMXErrorString(vmxerr)))

# Communication Classes
# SPI Class
class SPI:
    
    # local variables
    __spi_res_handle = 0
    
    def __init__(self, bitrate = 1000000):
        spi_clk = vmxzero.VMXChannelInfo(zero.getIO().GetSoleChannelIndex(vmxzero.SPI_CLK), vmxzero.SPI_CLK)
        spi_mosi = vmxzero.VMXChannelInfo(zero.getIO().GetSoleChannelIndex(vmxzero.SPI_MOSI), vmxzero.SPI_MOSI)
        spi_miso = vmxzero.VMXChannelInfo(zero.getIO().GetSoleChannelIndex(vmxzero.SPI_MISO), vmxzero.SPI_MISO)
        spi_cs = vmxzero.VMXChannelInfo(zero.getIO().GetSoleChannelIndex(vmxzero.SPI_CS), vmxzero.SPI_CS)
        spi_config = vmxzero.SPIConfig(bitrate) # default is 1mbs
        success, self.__spi_res_handle, vmxerr = zero.getIO().ActivateQuadchannelResource(spi_clk, spi_mosi, spi_miso, spi_cs, spi_config)
        if not success:
            print("Zero: Failed to initialize spi port")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully initialized spi port")
            
    def Write(self, data):
        tx_data = vmxzero.DirectBuffer(len(data))
        for i in range (tx_data):
            tx_data[i] = data[i]
        success, vmxerr = zero.getIO().SPI_Write(self.__spi_res_handle, tx_data, len(data))
        if not success:
            print("Zero: Failed to send data via spi")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
    
    def Read(self, size):
        rx_data = vmxzero.DirectBuffer(size)
        success, vmxerr = zero.getIO().SPI_Read(self.__spi_res_handle, rx_data, size)
        if not success:
            print("Zero: Failed to read data via spi")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            data = vmxzero.cdata(rx_data, size)
            return ord(data)
    
    def Transaction(self, data, size):
        tx_data = vmxzero.DirectBuffer(len(data))
        rx_data = vmxzero.DirectBuffer(size)
        for i in range (tx_data):
            tx_data[i] = data[i]
        success, vmxerr = zero.getIO().SPI_Transaction(self.__spi_res_handle, tx_data, rx_data, size)
        if not success:
            print("Zero: Failed transaction on spi")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            data = vmxzero.cdata(rx_data, size)
            return ord(data)

# UART Class
class UART:
    
    # local variables
    __uart_res_handle = 0
    
    def __init__(self, bitrate = 57600):
        uart_tx = vmxzero.VMXChannelInfo(zero.getIO().GetSoleChannelIndex(vmxzero.UART_TX), vmxzero.UART_TX)
        uart_rx = vmxzero.VMXChannelInfo(zero.getIO().GetSoleChannelIndex(vmxzero.UART_RX), vmxzero.UART_RX)
        uart_config = vmxzero.UARTConfig(bitrate)
        success, self.__uart_res_handle, vmxerr = zero.getIO().ActivateDualchannelResource(uart_tx, uart_rx, uart_config)
        if not success:
            print("Zero: Failed to initialize uart port")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully initialized uart port")
            
    def Write(self, data):
        tx_data = vmxzero.DirectBuffer(len(data))
        for i in range (tx_data):
            tx_data[i] = data[i]
        success, vmxerr = zero.getIO().UART_Write(self.__uart_res_handle, tx_data, len(data))
        if not success:
            print("Zero: Failed to send data via uart")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            
    def Read(self):
        # Check if there is something to read 
        success, bytes_available, vmxerr = zero.getIO().UART_GetBytesAvailable(self.__uart_res_handle)
        if not success:
            print("Zero: There was an error checking the bytes available to be read on uart")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            if bytes_available == 0:
                print("Zero: There is nothing to read on uart")
            else:
                rx_data = vmxzero.DirectBuffer(bytes_available)
                success, bytes_read, vmxerr = zero.getIO().UART_Read(self.__uart_res_handle, rx_data, bytes_available)
                if not success:
                    print("Zero: There was an error reading the bytes via uart")
                    print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
                else:
                    data = vmxzero.cdata(rx_data, bytes_read)
                    return ord(data)

# CAN Class
class CAN:
    
    # local variables
    __can_res_handle = 0
    
    def __init__(self, messageID = 0x0, messageMask = 0x0, maxMessages = 100):
        #Open CAN Port
        success, self.__can_res_handle, vmxerr = zero.getCAN().OpenReceiveStream(messageID, messageMask, maxMessages)
        if not success:
            print("Zero: Failed to initialize can port")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully initialized can port")
        #Enable Blackboard
        success, vmxerr = zero.getCAN().EnableReceiveStreamBlackboard(self.__can_res_handle, True)
        if not success:
            print("Zero: Failed to enable can blackboard")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully enabled can blackboard")
        #Flush RX FIFO
        success, vmxerr = zero.getCAN().FlushRxFIFO()
        if not success:
            print("Zero: Failed to flush can rx fifo")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully flushed can rx fifo")
        #Flush RX FIFO
        success, vmxerr = zero.getCAN().FlushTxFIFO()
        if not success:
            print("Zero: Failed to flush can tx fifo")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully flushed can tx fifo")
        #Set CAN Mode
        success, vmxerr = zero.getCAN().SetMode(vmxzero.VMXCAN.VMXCAN_NORMAL)
        if not success:
            print("Zero: Failed to set can to normal")
            print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
        else:
            print("Zero: Successfully set can to normal mode")
        #Delay 20 Milliseconds so CAN can stabilize.
        zero.getTime().DelayMilliseconds(20)
        
    def Write(self, address, data, periodMS = 0):
        msg = vmxzero.VMXCANMessage()
        tx_data = vmxzero.DirectBuffer(8)
        for x in range(8):
            tx_data.__setitem__(x, data[x])
        msg.messageID = address
        msg.dataSize = 8
        msg.setData(tx_data, 8)
        success, vmxerr = zero.getCAN().SendMessage(msg, periodMS)
        if not success:
            print("Zero: Failed to send CAN Message {}".format(vmxzero.GetVMXErrorString(vmxerr)))
    
    def Read(self, messageID):
        output = []
        rx_data = vmxzero.DirectBuffer(8)
        msg = vmxzero.VMXBlackboardTimestampedCANMessage()
        timeStamp = msg.sysTimeStampUS
        #success, vmxerr = zero.getCAN().GetBlackboardEntry(self.__can_res_handle, messageID, msg, timeStamp)
        #if not success:
            #print("Zero: Failed to read can blackboard")
            #print("Zero: Error {}".format(vmxzero.GetVMXErrorString(vmxerr)))
            #return -1
        #else:
        msg.getData(rx_data, msg.dataSize)
        b = vmxzero.cdata(rx_data, msg.dataSize)
        for j in b:
            output.append(ord(j))
        return output
        
# Titan Class        
class Titan(CAN):
    
    __ID = 42
    __DEVICE_TYPE = 0x2000000
    __MANUFACTURER_ID = 0xC0000
    __OFFSET = 64
    __BASE = __DEVICE_TYPE + __MANUFACTURER_ID + __ID
    
    __DISABLED_FLAG = __BASE
    __ENABLED_FLAG = __BASE + (__OFFSET * 1)
    __SET_MOTOR_SPEED = __BASE + (__OFFSET * 2)
    __DISABLE_MOTOR = __BASE + (__OFFSET * 3)
    __GET_TITAN_INFO = __BASE + (__OFFSET * 4)
    __RETURN_TITAN_INFO = __BASE + (__OFFSET * 5)
    __GET_UNIQUE_ID = __BASE + (__OFFSET * 6)
    __RETURN_WORD_1 = __BASE + (__OFFSET * 7)
    __RETURN_WORD_2 = __BASE + (__OFFSET * 8)
    __RETURN_WORD_3 = __BASE + (__OFFSET * 9)
    __CONFIG_MOTOR = __BASE + (__OFFSET * 10)
    __GET_MOTOR_FREQUENCY = __BASE + (__OFFSET * 11)
    __RETURN_MOTOR_FREQUENCY = __BASE + (__OFFSET * 12)
    __RESET_ENCODER = __BASE + (__OFFSET * 13)
    __SET_CURRENT_LIMIT = __BASE + (__OFFSET * 14)
    __SET_MOTOR_STOP_MODE = __BASE + (__OFFSET * 15)
    __SET_TARGET_VELOCITY = __BASE + (__OFFSET * 16)
    __SET_TARGET_DISTANCE = __BASE + (__OFFSET * 17)
    __RETURN_PID_TARGET_STATUS = __BASE + (__OFFSET * 19)
    __SET_PID_VALUES = __BASE + (__OFFSET * 20)
    __SET_PID_LIMITS = __BASE + (__OFFSET * 21)
    __ENCODER_0_COUNT = __BASE + (__OFFSET * 38)
    __ENCODER_1_COUNT = __BASE + (__OFFSET * 39)
    __ENCODER_2_COUNT = __BASE + (__OFFSET * 40)
    __ENCODER_3_COUNT = __BASE + (__OFFSET * 41)
    __RPM_0 = __BASE + (__OFFSET * 42)
    __RPM_1 = __BASE + (__OFFSET * 43)
    __RPM_2 = __BASE + (__OFFSET * 44)
    __RPM_3 = __BASE + (__OFFSET * 45)
    __LIMIT_SWITCH = __BASE + (__OFFSET * 46)
    __CURRENT = __BASE + (__OFFSET * 47)
    __MCU_TEMP = __BASE + (__OFFSET * 48)
    __ENCODER0_DIST_PER_TICK = 0
    __ENCODER1_DIST_PER_TICK = 0
    __ENCODER2_DIST_PER_TICK = 0
    __ENCODER3_DIST_PER_TICK = 0
    
    def __toSigned16(self, n):
        n = n & 0xFFFF
        return (n ^ 0x8000) - 0x8000
    
    def __toSigned32(self, n):
        n = n & 0xFFFFFFFF
        return (n ^ 0x80000000) - 0x80000000
    
    def __init__(self, canID = 42, frequency = 15600):
        if (canID > 0 and canID < 64):
            if (frequency >= 0 and frequency <= 20000):
                super().__init__()
                self.__ID = canID
                for x in range(4):
                    data = [x, frequency & 0xFF, frequency >> 8, 0, 0, 0, 0, 0]
                    self.Write(self.__CONFIG_MOTOR, data, 0)
                print("Zero: Started Titan Driver")
            else:
                print("Zero: Frequency set out of range")
        else:
            print("Zero: Invalid canID, range is 1 - 63")
        
    def Enable(self, enFlag):
        data = [0]*8
        if enFlag:
            self.Write(self.__ENABLED_FLAG, data, 10) # Repeat every 10ms
        else:
            self.Write(self.__DISABLED_FLAG, data, 10) # Repeat every 10ms
            
    def GetID(self):
        data = self.Read(self.__RETURN_TITAN_INFO)
        if (len(data) <= 0):
            print("Zero: Error reading GetID")
            return -1
        else:
            return data[0]
        
    def GetFrequency(self):
        data = self.Read(self.__RETURN_MOTOR_FREQUENCY)
        if (len(data) <= 0):
            print("Zero: Error reading Frequency")
            return -1
        else:
            return data[0] + (data[1] << 8)
    
    def GetFirmwareVersion(self):
        data = self.Read(self.__RETURN_TITAN_INFO)
        if (len(data) <= 0):
            print("Zero: Error reading Firmware Version")
            return -1
        else:
            return "Firmware Version: {}.{}.{}".format(data[1], data[2], data[3])
    
    def GetHardwareVersion(self):
        data = self.Read(self.__RETURN_TITAN_INFO)
        if (len(data) <= 0):
            print("Zero: Error reading Hardware Version")
            return -1
        else:
            if (data[4] == 1):
                return "Hardware: Titan Quad, Version: {}".format(data[5])
            elif(data[4] == 2):
                return "Hardware: Titan Small, Version: {}".format(data[5])
            else:
                return "No Hardware Found"
            
    def DisableMotor(self, motor):
        if (motor >= 0 and motor < 4):
            data = [motor, 0, 0, 0, 0, 0, 0, 0]
            self.Write(self.__DISABLE_MOTOR, data, 0)
        else:
            print("Disable Motor out of range")
    
    def GetControllerTemp(self):
        data = self.Read(self.__MCU_TEMP)
        if (len(data) <= 0):
            print("Zero: Error reading MCU Temp")
            return -1
        else:
            return data[0] + data[1]/100.0
    
    def SetMotorSpeed(self, motor, speed):
        if (motor >= 0 and motor < 4):
            if (speed <= 1.0 and speed >= -1.0):
                speed = speed * 100
                inA = 0
                inB = 0
                if (speed == 0):
                    inA = 1
                    inB = 1
                elif (speed > 0):
                    inA = 1
                    inB = 0
                elif (speed < 0):
                    inA = 0
                    inB = 1
                else:
                    inA = 0
                    inB = 0
                data = [motor, int(abs(speed)), inA, inB, 0, 0, 0, 0]
                self.Write(self.__SET_MOTOR_SPEED, data, 0)
            else:
                print("Set Speed out of range")
        else:
            print("Set Speed out of range motor")
        
    def GetLimitSwitch(self, motor, direction):
        data = self.Read(self.__LIMIT_SWITCH)
        if direction:
            index = (motor * 2) + 1
        else:
            index = (motor * 2)
        return data[index]
    
    def GetRPM(self, motor):
        if(motor >= 0 and motor < 4):
            if (motor == 0):
                data = self.Read(self.__RPM_0)
            elif (motor == 1):
                data = self.Read(self.__RPM_1)
            elif (motor == 2):
                data = self.Read(self.__RPM_2)
            elif (motor == 3):
                data = self.Read(self.__RPM_3)
            unsignRPM = data[0] | (data[1] << 8)
            rpm = self.__toSigned16(unsignRPM)
            return rpm
        else:
            print("Motor out of range for RPM check")
            return -1
    
    def GetSerialNumber(self):
        data_1 = self.Read(self.__RETURN_WORD_1)
        data_2 = self.Read(self.__RETURN_WORD_2)
        data_3 = self.Read(self.__RETURN_WORD_3)
        
        dWord1 = data_1[0] + (data_1[1] << 8) + (data_1[2] << 16) + (data_1[3] << 24)
        dWord2 = data_2[0] + (data_2[1] << 8) + (data_2[2] << 16) + (data_2[3] << 24)
        dWord3 = data_3[0] + (data_3[1] << 8) + (data_3[2] << 16) + (data_3[3] << 24)
        
        return f'{dWord1:0X} + {dWord2:0X} + {dWord3:0X}'
    
    def ConfigureEncoder(self, encoder, distPerTick):
        if (encoder == 0):
            self.__ENCODER0_DIST_PER_TICK = distPerTick
        elif (encoder == 1):
            self.__ENCODER1_DIST_PER_TICK = distPerTick
        elif (encoder == 2):
            self.__ENCODER2_DIST_PER_TICK = distPerTick
        elif (encoder == 3):
            self.__ENCODER3_DIST_PER_TICK = distPerTick
        else:
            print("Encoder out of range for configuration")
    
    def GetEncoderDistance(self, encoder):
        if (encoder == 0):
            data = self.Read(self.__ENCODER_0_COUNT)
            ticks = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
            distance = self.__toSigned32(ticks) * self.__ENCODER0_DIST_PER_TICK
            return distance
        elif (encoder == 1):
            data = self.Read(self.__ENCODER_1_COUNT)
            ticks = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
            distance = self.__toSigned32(ticks) * self.__ENCODER1_DIST_PER_TICK
            return distance
        elif (encoder == 2):
            data = self.Read(self.__ENCODER_2_COUNT)
            ticks = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
            distance = self.__toSigned32(ticks) * self.__ENCODER2_DIST_PER_TICK
            return distance
        elif (encoder == 3):
            data = self.Read(self.__ENCODER_3_COUNT)
            ticks = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
            distance = self.__toSigned32(ticks) * self.__ENCODER3_DIST_PER_TICK
            return distance
        else:
            print("Encoder out of range for distance")
            return -1
        
    def ResetEncoder(self, encoder):
        data = [encoder, 0, 0, 0, 0, 0, 0, 0]
        self.Write(self.__RESET_ENCODER, data, 0)