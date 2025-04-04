# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)



# --->>> Copy your convolve(a, b) and multiply(a, b) functions here.

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

def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""
    values = []
    start = max(a.start(), b.start())
    stop = min(a.stop(), b.stop())
    for i in range(start, stop):
        value = a.value(i) * b.value(i)
        values.append(value)
    distribution = Distribution(start, values)
    distribution.normalize()
    return distribution


if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
          drawstyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in range(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b',  drawstyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 50)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r',  drawstyle='steps')

    show()
