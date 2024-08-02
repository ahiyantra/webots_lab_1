"""line_follower_1 file."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

MAX_SPEED = 6.28
WHEEL_RADIUS = 0.0201  # meters
AXLE_LENGTH = 0.052  # meters
phildot = 0
phirdot = 0

# state machine variables
epuck_state = "following_line"
lap_completed = False
spin_direction = "right"

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

leftMotor=robot.getDevice('left wheel motor')
rightMotor=robot.getDevice('right wheel motor')

leftMotor.setPosition(float('Inf'))
rightMotor.setPosition(float('Inf'))

gs=[]
for i in range(3):
    gs.append(robot.getDevice('gs'+str(i)))
    gs[-1].enable(timestep)
    
# Odometry variables
total_distance = 0
total_rotation = 0

# Initialize world coordinates
xw = 0
yw = 0.028  # Slightly in front of the start line
alpha = 0  # Initial orientation

print("E-puck has started following the track's line path.")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    
    # white = higher (500~700)
    # black = lower (200~400)
    
    ga = []
    for gsensor in gs:
        ga.append(gsensor.getValue())
    print(ga)
    
    # Calculate error
    error = math.sqrt(xw**2 + yw**2)
    
    # Line following logic with smooth turns
    if epuck_state == "following_line":
        # Existing line following logic here
        if (total_distance>=3.04 and error <= 0.2): # possibly, it's at start/finish line
            lap_completed = True
            epuck_state = "stopped"
            phildot, phirdot = 0.0, 0.0
        elif (ga[0] > 500 and ga[1] < 350 and ga[2] > 500): # drive straight (white/black/white)
            phildot, phirdot = MAX_SPEED, MAX_SPEED
        elif (ga[1] < 350 and ga[2] < 350):  # turn right (white/black/black) 
            phildot, phirdot = 0.2*MAX_SPEED, -0.1*MAX_SPEED
            spin_direction = "right"
        elif (ga[1] < 350 and ga[0] < 350):  # turn left (black/black/white) 
            phildot, phirdot = -0.1*MAX_SPEED, 0.2*MAX_SPEED
            spin_direction = "left"
        elif (ga[0]>500 and ga[1]>500 and ga[2]>500): # spin one way (white/white/white)
            if (spin_direction == "right"):
                phildot, phirdot = 0.2*MAX_SPEED, -0.1*MAX_SPEED
            else:
                phildot, phirdot = -0.1*MAX_SPEED, 0.2*MAX_SPEED
    elif epuck_state == "stopped":    
        print(f"Current distance: {total_distance:.4f} meters")
        print(f"Current rotation: {math.degrees(alpha):.2f} degrees")
        print(f"Current position: ({xw:.4f}, {yw:.4f})")
        print(f"Error from start: {error:.4f} meters")
        print("E-puck has completed the track & returned to start/finish line.")
        break
        
    #if (total_distance>=3.055): # stop (start line)
    #    phildot, phirdot = 0.0, 0.0
    #elif (ga[0]>500 and ga[1]<350 and ga[2]>500): # drive straight (white/black/white)
    #    phildot, phirdot = MAX_SPEED, MAX_SPEED
    #elif (ga[1]<350 and ga[2]<350) : # turn right (white/black/black) # ga[0]>500 and 
    #    phildot, phirdot = 0.2*MAX_SPEED, -0.1*MAX_SPEED
    #elif (ga[1]<350 and ga[0]<350): # turn left (black/black/white) #  and ga[2]>500 
    #    phildot, phirdot = -0.1*MAX_SPEED, 0.2*MAX_SPEED
    #elif (ga[0]>500 and ga[1]>500 and ga[2]>500): # spin right (white/white/white)
    #    phildot, phirdot = 0.2*MAX_SPEED, -0.1*MAX_SPEED

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    # Set motor velocities
    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)
    
    # Odometry calculations
    delta_t = timestep / 1000.0  # Convert timestep to seconds
    
    # Calculate displacement
    delta_x = (WHEEL_RADIUS * (phildot + phirdot) / 2) * delta_t
    total_distance += delta_x
    
    # Calculate rotation
    delta_omega = (WHEEL_RADIUS * (phirdot - phildot) / AXLE_LENGTH) * delta_t
    total_rotation += delta_omega
    
    # Update world coordinates
    xw = xw + math.cos(alpha) * delta_x
    yw = yw + math.sin(alpha) * delta_x
    alpha = alpha + delta_omega
    
    # Print odometry information
    print(f"Current distance: {total_distance:.4f} meters")
    print(f"Current rotation: {math.degrees(alpha):.2f} degrees")
    print(f"Current position: ({xw:.4f}, {yw:.4f})")
    print(f"Error from start: {error:.4f} meters")
    
    pass

# Enter here exit cleanup code.
