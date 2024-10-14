import numpy as np
import cv2
import time
import math
import random

# Simulation settings
simulationTime = 0.005  # Time step for world updates (200 Hz)
blackImg = np.zeros((800, 800, 3))  # Empty image for visualization

# World state variables (global frame)
xWorld = 20  # Initial x-coordinate of the robot in the world
yWorld = 200  # Initial y-coordinate of the robot in the world
yawWorld = -0.3  # Initial orientation of the robot in radians
steerNoise = 0  # Steering noise
lastSteer = 0  # Last steering angle
tSteer = 0  # Time when steering was last updated
nowSteer = 0  # Current steering angle

# Robot state variables (local frame)
xRobot = 20  # Initial x-coordinate of the robot in local coordinates
yRobot = 200  # Initial y-coordinate of the robot in local coordinates
yawRobot = -0.3  # Initial orientation of the robot in local frame

# Control and motion parameters
maxSteer = 29  # Maximum steering angle
steer = 0  # Current steering angle
velocity = 60  # Velocity of the robot in units per second
robotLength = 26  # Length of the robot (used for turning radius calculations)

# PID control variables
eI = list()  # List to accumulate integral errors
last_eA = 0  # Last error in angle for derivative calculation

def real_world():
    """
    Update the robot's position and orientation in the global (world) frame
    based on the steering angle and velocity.
    """
    global xWorld, yWorld, yawWorld, steer, lastSteer, nowSteer

    # Simulate smooth steering transition using an exponential function
    nowSteer = steerNoise - (steerNoise - lastSteer) * (math.e ** (-1.6 * (time.time() - tSteer)))
    
    # Update yaw (orientation) based on velocity, steering angle, and robot length
    yawWorld += (velocity / robotLength * math.tan(math.radians(nowSteer)) * simulationTime)
    
    # Update global position based on velocity and yaw
    xWorld += velocity * math.cos(yawWorld) * simulationTime
    yWorld += velocity * math.sin(yawWorld) * simulationTime

def robot_localization():
    """
    Update the robot's position and orientation in the local (robot) frame
    to simulate sensor/odometry updates at lower frequency.
    """
    global xRobot, yRobot, yawRobot
    
    # Update local yaw to be an average of global yaw and local yaw (for smoother transition)
    yawRobot = (yawWorld + yawRobot) / 2
    
    # Update local position based on velocity and global yaw (approximation)
    xRobot += velocity * math.cos(yawWorld) * simulationTime * 6
    yRobot += velocity * math.sin(yawWorld) * simulationTime * 6

def robot_actuate(st):
    """
    Set the robot's steering angle with some added noise to simulate real-world conditions.
    """
    global steer, steerNoise, lastSteer, tSteer, nowSteer
    
    lastSteer = nowSteer  # Update last known steer
    steer = st  # Set the new steering angle
    steerNoise = st + random.randint(-3, 3)  # Add random noise to the steering angle
    tSteer = time.time()  # Record the time of this steering actuation

def draw():
    """
    Visualize the robot's movement on the screen using OpenCV.
    """
    global blackImg

    # Draw the robot's local and global positions
    blackImg = cv2.circle(blackImg, (int(xRobot), int(yRobot)), 1, (255, 0, 0), 1)
    blackImg = cv2.circle(blackImg, (int(xWorld), int(yWorld)), 1, (0, 255, 0), 1)
    
    # Display the image
    cv2.imshow("localization", blackImg)
    cv2.waitKey(1)

def robot_head_pos(d=40):
    """
    Calculate the robot's head position, which is a point `d` units ahead of the current position.
    """
    xHead = xWorld + d * math.cos(yawWorld)
    yHead = yWorld + d * math.sin(yawWorld)
    return xHead, yHead

def perpendicular_line():
    """
    Calculate the slope of a line perpendicular to the robot's current yaw.
    """
    xHead, yHead = robot_head_pos()
    slope = math.tan(math.radians(90 - math.degrees(yawWorld)))
    return xHead, yHead, slope

def feedback():
    """
    Calculate the distance and angle errors to the target point for feedback control.
    """
    _, distance = robot_head_pos(d=40)
    
    if 5.6 < time.time() - t0 <= 6:
        distance -= 300
        distance *= -1
    else:
        distance -= 220
        distance *= -1

    if 5.6 < time.time() - t0 <= 6:
        angle = -50
    else:
        angle = -1 * math.degrees(yawWorld)

    return [distance, angle]

def robot_control(st=0):
    """
    Implement the Stanley controller to adjust the robot's steering angle based on
    the cross-track error and heading error.
    """
    global last_eA

    # Control parameters for the Stanley controller
    K_GLC = 1.16
    K_s = 20
    K_P_angle = 0.525
    K_D_angle = 1.5

    # Get feedback errors (distance and angle)
    eD, eA = feedback()
    
    # Stanley controller formula for steering angle
    st = K_P_angle * eA + K_D_angle * (eA - last_eA) + math.degrees(math.atan((K_GLC * eD) / (velocity + K_s)))
    
    # Update the last angle error for the derivative term
    last_eA = eA

    # Limit the steering angle to within the allowable range
    if st >= maxSteer:
        st = maxSteer
    elif st <= -maxSteer:
        st = -maxSteer

    # Actuate the steering with the calculated value
    robot_actuate(st)

# Timing variables to manage different update frequencies
t0 = time.time()
t1 = time.time()
t2 = time.time()
t3 = time.time()
t4 = time.time()

score = 0  # Variable to track the robot's score or progress

# Main simulation loop
while True:
    t = time.time()

    # Real world updates (every 5ms)
    if t - t1 >= simulationTime:
        real_world()
        t1 = time.time()

    # Localization updates (every 30ms)
    if t - t2 >= simulationTime * 6:
        robot_localization()
        t2 = time.time()

    # Control updates (every 250ms)
    if t - t3 >= simulationTime * 50:
        robot_control()  # Adjust steering based on control algorithm
        t3 = time.time()

    # Visualization updates (every 100ms)
    if t - t4 >= simulationTime * 20:
        draw()  # Draw the robot's current position
        t4 = time.time()

    # Update score based on progress along the y-axis
    score += yWorld - 220
    print(score)  # Display the current score
