"""basic_epuck_controller file."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

motor_left=robot.getDevice('left wheel motor')
motor_right=robot.getDevice('right wheel motor')

motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))

ds=[]
for i in range(8):
    ds.append(robot.getDevice('ps'+str(i)))
    ds[-1].enable(timestep)

ls=[]
for i in range(8):
    ls.append(robot.getDevice('ls'+str(i)))
    ls[-1].enable(timestep)

print("All sensors are now enabled!")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    dvalues=[]
    for dist in ds:
        dvalues.append(dist.getValue())
    print("dvalues =")
    print(dvalues)
    
    lvalues=[]
    for lise in ls:
        lvalues.append(lise.getValue())
    print("lvalues =")
    print(lvalues)

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    #motor_left.setVelocity(-6.28)
    #motor_right.setVelocity(6.28)
    
    motor_left.setVelocity(lvalues[7]/1000.0)
    motor_right.setVelocity(lvalues[0]/1000.0)

    pass

# Enter here exit cleanup code.
