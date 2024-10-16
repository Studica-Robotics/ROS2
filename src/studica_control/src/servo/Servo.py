from Teddy import*

# Define Objects
Ted = Teddy()
Servo0 = Servo(Port.SERVO0)

# Initialize Method
# Put all the Initialize code in here
# Runs once then passes to the Init_Loop
def _Init():
    pass

# Initialize Loop
# This will loop at 20ms until the start button is pressed
def _Init_Loop():
    pass

# Run Initialize
# Put any code in here that must be run once before the Run_Loop is called
def _Run_Init():
    pass

# Run Loop
# This will loop at 20ms unitl the stop button is pressed
def _Run_Loop():
    Servo0.SetAngle(300) # Set angle of servo to 100

# Stop
# This is called right after the stop button is pressed 
# Runs once then returns to waiting for Init  
def _Stop():
    pass

Ted.Init = _Init
Ted.Init_Loop = _Init_Loop
Ted.Run_Init = _Run_Init
Ted.Run_Loop = _Run_Loop
Ted.Stop = _Stop
Ted.Start()