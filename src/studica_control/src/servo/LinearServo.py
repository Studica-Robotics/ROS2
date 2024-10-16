from Teddy import*

# Define Objects
Ted = Teddy()

# Change the defualt values to be 0 for retracted and 100 for fully extended
Servo0 = Servo(Port.SERVO0, min = 0, max = 100)

# Initialize Method
# Put all the Initialize code in here
# Runs once then passes to the Init_Loop
def _Init():
    # Set the min and max bounds based on ms
    Servo0.SetBounds(min = 0.9, max = 2.1)

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
    Servo0.SetAngle(100) # Fully Extend the linear servo

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