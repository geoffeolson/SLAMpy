# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, mm_per_tick, robot_width,
                scanner_displacement):
    x, y, heading = old_pose
    left, right = motor_ticks
    left = left * mm_per_tick
    right = right * mm_per_tick
    heading_change = (right - left) / robot_width

    if heading_change == 0:
        # The Robot did not Turn. It just drove straight.
        x = x + left * cos(heading)
        y = y + left * sin(heading)
        ##############################################################
        # Think about if you need to modify your old code due to the
        # scanner displacement?
        #################################################################

    else:
        # The Robot Turned
        x = x - scanner_displacement * cos(heading)
        y = y - scanner_displacement * sin(heading)
        radius = left / heading_change + robot_width / 2.
        center_x = x - radius * sin(heading)
        center_y = y + radius * cos(heading)
        heading = (heading + heading_change) % (2 * pi)
        x = center_x + radius * sin(heading)
        y = center_y - radius * cos(heading)
        x = x + scanner_displacement * cos(heading)
        y = y + scanner_displacement * sin(heading)

        ################################################################
        # 1) Modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        # 2) Execute your old code, which implements the motion model
        #   for the center of the robot.
        # 3) Modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.
        ##############################################################

    return (x, y, heading)

if __name__ == '__main__':
    import os
    os.chdir("Unit_A")
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        x, y, heading = pose
        line = "F " + str(x) + " " + str(y) + " " + str(heading)
        f.write(line + "\n")
        print(line)
    f.close()
