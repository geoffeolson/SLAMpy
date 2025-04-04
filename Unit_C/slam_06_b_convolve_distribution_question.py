# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
from multiprocessing import Value
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

def convolve_old(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    values = []
    start = b.start() - a.stop()
    stop = b.stop() - a.start()
    center = a.start() + b.start()
    a = move(a, start)
    for i in range(0, stop - start):
        value = 0.0
        a = move(a, 1)
        for j in range(a.start(), a.stop()):
            value += a.value(j) * b.value(j)
        values.append(value)
    distribution = Distribution(center, values)
    distribution.normalize()
    return distribution

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    values = []
    start = a.start() + b.start()
    stop = a.stop() + b.stop()
    for i in range(start, stop):
        value = 0.0
        for j in range(a.start(), a.stop()):
            value += a.value(j) * b.value(i - j)
        values.append(value)
    distribution = Distribution(start, values)
    distribution.normalize()
    return distribution


if __name__ == '__main__':
    arena = (0,1000)

    # Move 3 times by 20.
    moves = [20] * 50

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.unit_pulse(10)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         drawstyle='steps')

    # Now move and plot.
    for m in moves:
        move_distribution = Distribution.triangle(m, 2)
        position = convolve(position, move_distribution)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             drawstyle='steps')
    ylim(0.0, 1.1)
    show()
