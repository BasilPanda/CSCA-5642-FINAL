"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import numpy as np
from controller import Robot

# create the Robot instance.
robot = Robot()
MAX_SPEED = 6.28
HALF_SPEED = MAX_SPEED / 2
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

gs = []
for i in range(3):
    gs.append(robot.getDevice(f"gs{i}"))
    gs[-1].enable(timestep)

g = []
for g_sensor in gs:
    g.append(g_sensor.getValue())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(HALF_SPEED)
right_motor.setVelocity(HALF_SPEED)

# Initialize odometer variables
distance = 0.0
orientation = 0.0  # Orientation in radians
r = 0.0201
dist_wheels = 0.052
t = timestep / 1000

# For smoothing on turns
prev_left_speed = 0.0
prev_right_speed = 0.0
smooth_factor = 0.1

xw = 0
yw = 0.028
alpha = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    g = []
    for g_sensor in gs:
        g.append(g_sensor.getValue())
    #print(g)
    # Process sensor data here.
    # Line following
    if g[0] > 500 and g[1] < 350 and g[2] > 500: # go forward
        left_speed, right_speed  = MAX_SPEED, MAX_SPEED
    elif g[2] < 550: # turn right
        left_speed, right_speed  =  0.25 * MAX_SPEED, -0.1 * MAX_SPEED
    elif g[0] < 550: # turn left
        left_speed, right_speed  =  -0.1 * MAX_SPEED, 0.25 * MAX_SPEED

    smoothed_left = (1 - smooth_factor) * prev_left_speed + smooth_factor * left_speed
    smoothed_right = (1 - smooth_factor) * prev_right_speed + smooth_factor * right_speed
    prev_left_speed = smoothed_left
    prev_right_speed = smoothed_right

    delta_x = (r*left_speed + r*right_speed) * t / 2
    delta_theta = ((r*right_speed - r*left_speed) / dist_wheels * t)
    # Odometry
    distance += delta_x
    orientation += delta_theta
    offset = np.pi/2
    xw += np.cos(alpha + offset) * delta_x
    yw += np.sin(alpha + offset) * delta_x
    alpha += delta_theta

    print(f"xw: {xw:.2f} yw: {yw:.2f} alpha:{alpha}")
    print(f"Euclidean distance: {np.sqrt(xw**2 + yw**2)}")
    print(f"Distance: {distance}")
    print(f"Orientation Degrees: {orientation/3.14 * 180}")
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    pass

# Enter here exit cleanup code.
