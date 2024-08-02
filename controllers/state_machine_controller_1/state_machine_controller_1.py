"""state_machine_controller_1 file."""

def sleep(duration):
    # Waits for duration seconds before returning
    global robot
    end_time = robot.getTime() + duration
    while robot.step(TIME_STEP) != -1 and robot.getTime() < end_time:
        pass

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot, DistanceSensor, Motor

# create the Robot instance.

robot = Robot()

# get the time step of the current world.

TIME_STEP = int(robot.getBasicTimeStep())

# initialize devices

ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

current_state = 0

# state 0 = first forward
# state 1 = rotate 180 degrees, counter-clockwise
# state 2 = second forward
# state 3 = rotate clockwise, degrees don't matter
# state 4 = third forward
# state 5 = stop

# Main loop:
# - perform simulation steps until Webots is stopping the controller

# The professor said that an intensity value of 80 is good for this lab.
while robot.step(TIME_STEP) != -1:

    # Read the sensors:
    
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    print(psValues)

    # Process sensor data here.
    
    if current_state == 0: # first forward
        if psValues[0] < 80:
            leftMotor.setVelocity(3.0)
            rightMotor.setVelocity(3.0)
        else:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            current_state = 1
    elif current_state == 1: # rotate 180 degrees, counter-clockwise
        leftMotor.setVelocity(-3.14)
        rightMotor.setVelocity(3.14)
        sleep(1.42) # 1.569
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        current_state = 2
    elif current_state == 2: # second forward
        if psValues[0] < 80:
            leftMotor.setVelocity(3.0)
            rightMotor.setVelocity(3.0)
        else: 
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            current_state = 3
    elif current_state == 3: # rotate clockwise, degrees don't matter
        if psValues[5] < 80: 
            leftMotor.setVelocity(3.14)
            rightMotor.setVelocity(-3.14)
            sleep(0.7)
        else: 
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            current_state = 4
    elif current_state == 4: # third forward
        if psValues[5] >= 80:
            leftMotor.setVelocity(3.0)
            rightMotor.setVelocity(3.0)
        else: 
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            current_state = 5
    elif current_state == 5: # stop
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
    
    pass

# Enter here exit cleanup code.
