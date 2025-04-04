# Implement the first move model for the Lego robot.
# 02_a_filter_motor
# Claus Brenner, 31 OCT 2012
from pylab import *
from lego_robot import *
from math import sin,cos,pi

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, mm_per_tick, robot_width):
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
        radius = left / heading_change + robot_width / 2.
        center_x = x - radius * sin(heading)
        center_y = y + radius * cos(heading)
        heading = (heading + heading_change) % (2 * pi)
        x = center_x + radius * sin(heading)
        y = center_y - radius * cos(heading)

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
    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Start at origin (0,0), looking along x axis (alpha = 0).
    pose = (0.0, 0.0, 0.0)

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width)
        filtered.append(pose)

    # Draw result.
    for pose in filtered:
        print( pose)
        plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')
    show(block=True)
