"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor
import numpy as np

# import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
# TODO: Fill this in with a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 500

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.126  # ePuck's maximum wheel speed in m/s
print("EPUCK_MAX_WHEEL_SPEED: ", EPUCK_MAX_WHEEL_SPEED)
MAX_SPEED = 6.28

# Calculate the radius of the wheel
wheel_radius = EPUCK_MAX_WHEEL_SPEED / MAX_SPEED
print("Wheel Radius: ", wheel_radius)

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())
print("SIM_TIMESTEP: ", SIM_TIMESTEP)

# Initialize Motors
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Enable motor position sensors (encoders)
left_position_sensor = leftMotor.getPositionSensor()
right_position_sensor = rightMotor.getPositionSensor()
left_position_sensor.enable(SIM_TIMESTEP)
right_position_sensor.enable(SIM_TIMESTEP)
# claude 3.5 sonnet informed us about the position sensor

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [
    robot.getDevice("gs0"),
    robot.getDevice("gs1"),
    robot.getDevice("gs2"),
]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10):
    robot.step(SIM_TIMESTEP)

# Variables for speed measurement
start_position = 0
start_time = 0
start_line_counter = 0
start_counter_threshold = 4

base_speed = MAX_SPEED * 0.5
vL = base_speed
vR = base_speed


# update_odometry function
# vL is the
def update_odometry(vL, vR, time_delta):
    global pose_x, pose_y, pose_theta

    # Calculate instantaneous wheel speeds (assumed in rad/s)
    l_wheel_speed = vL
    r_wheel_speed = vR

    # Compute linear and angular velocities
    linear_velocity = ((l_wheel_speed + r_wheel_speed) * wheel_radius) / 2.0  # m/s
    angular_velocity = (
        (r_wheel_speed - l_wheel_speed) * wheel_radius
    ) / EPUCK_AXLE_DIAMETER  # rad/s

    delta_s = linear_velocity * time_delta
    delta_theta = angular_velocity * time_delta

    # Update pose using midpoint integration for x and y
    pose_x += delta_s * math.cos(pose_theta + delta_theta / 2)
    pose_y += delta_s * math.sin(pose_theta + delta_theta / 2)
    pose_theta = (pose_theta + delta_theta) % (2 * math.pi)

    return pose_x, pose_y, pose_theta


def ramp_speed_func(curr_speed, target_speed, ramp_rate):
    return curr_speed + ramp_rate * (target_speed - curr_speed)


ramp_speed = np.vectorize(ramp_speed_func)

# Main Control Loop:
controller_state = "line_follower"
is_prev_turning_left = False

corner_speed_multiplier = 1

while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # print(gsr) # TODO: Uncomment to see the ground sensor values!

    if controller_state == "speed_measurement":
        # Start measurement
        if start_time == 0:
            start_time = robot.getTime()
            start_position = left_position_sensor.getValue()
            vL = MAX_SPEED
            vR = MAX_SPEED

        # After 6 seconds, calculate speed
        if robot.getTime() - start_time >= 6.0:
            end_position = left_position_sensor.getValue()
            # Convert radians to meters (wheel radius = 0.02 meters)
            distance = (end_position - start_position) * 0.02
            EPUCK_MAX_WHEEL_SPEED = distance / 6.0  # Speed = Distance/Time
            print(f"Measured max speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
            controller_state = "line_follower"  # Switch to next state
            vL = 0
            vR = 0
        else:
            vL = MAX_SPEED
            vR = MAX_SPEED

    # Hints:
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #

    # What ChatGPT did:
    # - Help normalise the theta, and used time delta in calculating velocities
    # - Add comments to the code
    # - Organisation of code
    # - Did the midpoint integraption method instead of using the rotation matrix

    # Original update_odometry function:
    # def update_odometry(vL, vR, time_delta):
    # global pose_x, pose_y, pose_theta

    # # Calculate instantaneous wheel speeds (assumed in rad/s)
    # l_wheel_speed = vL
    # r_wheel_speed = vR

    # # Compute linear and angular velocities
    # linear_velocity = ((l_wheel_speed + r_wheel_speed) * wheel_radius) / 2.0  # m/s
    # angular_velocity = (
    #     (r_wheel_speed - l_wheel_speed) * wheel_radius
    # ) / EPUCK_AXLE_DIAMETER  # rad/s

    # # Multiply by time_delta to get the incremental displacement and rotation
    # delta_x = linear_velocity * time_delta  # displacement in meters
    # delta_theta = angular_velocity * time_delta  # rotation in radians

    # # Create a relative motion vector in the robot's (body) frame.
    # relative_motion = np.array([[delta_x], [0], [delta_theta]])

    # # Construct a rotation matrix to transform the relative motion to the world frame
    # rotation_matrix = np.array(
    #     [
    #         [np.cos(pose_theta), -np.sin(pose_theta), 0],
    #         [np.sin(pose_theta), np.cos(pose_theta), 0],
    #         [0, 0, 1],
    #     ]
    # )

    # # Use matrix multiplication to compute the motion in the inertial (world) frame.
    # inertial_motion = np.matmul(rotation_matrix, relative_motion).flatten()

    # # Update the pose values
    # pose_x += inertial_motion[0]
    # pose_y += inertial_motion[1]
    # pose_theta += inertial_motion[2]

    # # Normalize pose_theta so that it remains within [0, 2*pi)
    # pose_theta %= 2 * math.pi

    # return pose_x, pose_y, pose_theta
    elif controller_state == "line_follower":
        # Use a fixed base speed (tweak this as necessary; using a fraction of MAX_SPEED often yields smoother behavior)

        # Determine sensor conditions. We consider that a sensor "detects" the line if its reading is below the threshold.
        left_detected = gsr[LEFT_IDX] <= GROUND_SENSOR_THRESHOLD
        center_detected = gsr[CENTER_IDX] <= GROUND_SENSOR_THRESHOLD
        right_detected = gsr[RIGHT_IDX] <= GROUND_SENSOR_THRESHOLD

        # Adjust speeds based on which sensor sees the line.
        if left_detected and not right_detected:
            # Line is to the left, so turn right by reducing left motor speed.
            # vL = base_speed * 0.2 * corner_speed_multiplier  # reduce left speed
            # vR = base_speed * 0.5 * corner_speed_multiplier  # increase right speed
            is_prev_turning_left = True
            # corner_speed_multiplier = max(corner_speed_multiplier * 0.9, 0.5)
            vL = ramp_speed(vL, base_speed * 0.0, 0.10)
            vR = ramp_speed(vR, base_speed * 0.4, 0.10)
        elif right_detected and not left_detected:
            # Line is to the right, so turn left by reducing right motor speed.
            # vL = base_speed * 0.5 * corner_speed_multiplier
            # vR = base_speed * 0.2 * corner_speed_multiplier
            is_prev_turning_left = False
            # corner_speed_multiplier = max(corner_speed_multiplier * 0.9, 0.5)
            vL = ramp_speed(vL, base_speed * 0.4, 0.10)
            vR = ramp_speed(vR, base_speed * 0.0, 0.10)
        elif center_detected and not (left_detected or right_detected):
            # When only center sensor sees the line, go straight.
            # vL = base_speed * corner_speed_multiplier
            # vR = base_speed * corner_speed_multiplier
            # corner_speed_multiplier = min(corner_speed_multiplier * 1.05, 1)
            vL = ramp_speed(vL, base_speed, 0.10)
            vR = ramp_speed(vR, base_speed, 0.10)
        elif not (left_detected or center_detected or right_detected):
            # If no line is detected, turn in place (here turning left) to search for the line.
            if is_prev_turning_left:
                # vL = -base_speed * 0
                # vR = base_speed * 0.4
                vL = ramp_speed(vL, base_speed * -0.3, 0.08)
                vR = ramp_speed(vR, base_speed * 0.5, 0.08)
            else:
                # vL = base_speed * 0
                # vR = -base_speed * 0.4
                vL = ramp_speed(vL, base_speed * 0.5, 0.08)
                vR = ramp_speed(vR, base_speed * -0.3, 0.08)

        # Check if all three sensors detect the line for a sustained period which can indicate a start line (or loop closure).
        if left_detected and center_detected and right_detected:
            start_line_counter += 1
            if start_line_counter >= start_counter_threshold:
                print("start line detected")
                # Reset odometry as needed.
                pose_x = 0
                pose_y = 0
                pose_theta = 0
        else:
            start_line_counter = 0

    # TODO: Call update_odometry Here

    # Hints:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second.
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)

    time_delta = SIM_TIMESTEP / 1000
    pose_x, pose_y, pose_theta = update_odometry(vL, vR, time_delta)

    # TODO: Insert Loop Closure Code Here

    # Hints:
    #
    # 1) Set a flag whenever you encounter the line
    #
    # 2) Use the pose when you encounter the line last
    # for best results

    print(
        "Current pose: [%5f, %5f, %5f], vL: %5f, vR: %5f"
        % (pose_x, pose_y, pose_theta, vL, vR)
    )
    vL = np.clip(vL, -MAX_SPEED, MAX_SPEED)
    vR = np.clip(vR, -MAX_SPEED, MAX_SPEED)
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
