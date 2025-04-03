# Print the increments of the left and right motor.
# Now using the LegoLogfile class.
# 01_b_print_motor_increments.py
# Claus Brenner, 07 NOV 2012
from numpy import block
from pylab import plot
from pylab import *
from lego_robot import LegoLogfile

if __name__ == '__main__':
    import os
    os.chdir("Unit_A")
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    for i in range(20):
        print(logfile.motor_ticks[i])

    plot(logfile.motor_ticks)
    show(block=True)
