# Implement the first move model for the Lego robot.
# 02_a_filter_motor
# Claus Brenner, 31 OCT 2012
from pylab import *
from lego_robot import *
from math import sin,cos,pi

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):
    x, y, theta = old_pose
    l, r = motor_ticks
    l = l * ticks_to_mm
    r = r * ticks_to_mm
    alpha = (r - l) / robot_width

    if alpha == 0: # was there is a turn?
        # No turn. Just drive straight.
        x = x + l * cos(theta)
        y = y + l * sin(theta)
        ad = 4

    else:
        # turned
        R = l / alpha
        Rc = R + robot_width / 2.0
        cx = x - Rc * sin(theta)
        cy = y + Rc * cos(theta)
        theta = (theta + alpha) % (2 * pi)
        x = cx + Rc * sin(theta)
        y = cy - Rc * cos(theta)
        ass=45

    return (x, y, theta)

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
