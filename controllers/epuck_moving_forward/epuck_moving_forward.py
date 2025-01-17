"""epuck_moving_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# get the motor devices
#leftMotor = robot.getDevice('left wheel motor')
#rightMotor = robot.getDevice('right wheel motor')
# set the target position of the motors
#leftMotor.setPosition(10.0)
#rightMotor.setPosition(10.0)

# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# set up the motor speeds at 10% of the MAX_SPEED.
#leftMotor.setVelocity(0.1 * MAX_SPEED)
#rightMotor.setVelocity(0.1 * MAX_SPEED)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    leftMotor.setVelocity(6.28)
    rightMotor.setVelocity(6.28)
    
    #leftMotor.setPosition(6.28)
    #rightMotor.setPosition(0)
    #leftMotor.setPosition(6.28)
    #rightMotor.setPosition(6.28)
    
    #pass

# Enter here exit cleanup code.
