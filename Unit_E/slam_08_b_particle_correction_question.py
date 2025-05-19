# The particle filter, prediciton and correction.
#
# slam_08_b_particle_correction.
# Claus Brenner, 04.01.2013
from lego_robot import *
from slam_e_library import get_cylinders_from_scan, assign_cylinders
from math import sin, cos, pi, atan2, sqrt
import random
from scipy.stats import norm as normal_dist
import numpy as np
import os
os.chdir("Unit_E")


class ParticleFilter:
    def __init__(self, initial_particles,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The particles.
        self.particles = initial_particles

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

    # State transition. This is exactly the same method as in the Kalman filter.
    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return (g1, g2, g3)

    def predict(self, control):
        """The prediction step of the particle filter."""
        # --->>> Insert code from previous question here.
        left, right = control
        # Compute left and right variance.
        # alpha_1 is self.control_motion_factor.
        # alpha_2 is self.control_turn_factor.
        # Then, do a loop over all self.particles and construct a new
        # list of particles.
        # In the end, assign the new list of particles to self.particles.
        # For sampling, use random.gauss(mu, sigma). (Note sigma in this call
        # is the standard deviation, not the variance.)
        a1 = self.control_motion_factor
        a2 = self.control_turn_factor
        Gl2 = (a1 * left)**2 + (a2 * (left - right))**2
        Gr2 = (a1 * right)**2 + (a2 * (left - right))**2
        for i in range(len(self.particles)):
            # Sample from the normal distribution.
            left_ = random.gauss(left, sqrt(Gl2))
            right_ = random.gauss(right, sqrt(Gr2))
            # Compute new particle position.
            self.particles[i] = ParticleFilter.g(self.particles[i], (left_, right_), self.robot_width)


    # Measurement. This is exactly the same method as in the Kalman filter.
    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           corresponding (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return (r, alpha)

    def probability_of_measurement(self, measurement, predicted_measurement):
        """Given a measurement and a predicted measurement, computes
           probability."""
        # Compute differences to real measurements.

        # --->>> Compute difference in distance and bearing angle.
        # Important: make sure the angle difference works correctly and does
        # not return values offset by 2 pi or - 2 pi.
        # You may use the following Gaussian PDF function:
        # scipy.stats.norm.pdf(x, mu, sigma). With the import in the header,
        # this is normal_dist.pdf(x, mu, sigma).
        # Note that the two parameters sigma_d and sigma_alpha discussed
        # in the lecture are self.measurement_distance_stddev and
        # self.measurement_angle_stddev.
        d = measurement[0] - predicted_measurement[0]
        a = measurement[1] - predicted_measurement[1]
        a = (a + pi) % (2 * pi) - pi  # Normalize angle to [-pi, pi] range.
        Pd = normal_dist.pdf(d, 0, self.measurement_distance_stddev)
        Pa = normal_dist.pdf(a, 0, self.measurement_angle_stddev)
        return Pd * Pa

    def compute_weights(self, cylinders, landmarks):
        """Computes one weight for each particle, returns list of weights."""
        weights = []
        for robot_pose in self.particles:
            # Get list of tuples:
            # [ ((range_0, bearing_0), (landmark_x, landmark_y)), ... ]
            assignment = assign_cylinders(cylinders, robot_pose, self.scanner_displacement, landmarks)
            # This will require a loop over all (measurement, landmark)
            # in assignment. Append weight to the list of weights.
            weight = 1.0
            n = len(assignment)
            for measurement, landmark in assignment:
                # Compute the predicted measurement.
                predicted_measurement = ParticleFilter.h( robot_pose, landmark, self.scanner_displacement)
                # weight is the product of probabilities for each landmark measurment.
                weight *= self.probability_of_measurement( measurement, predicted_measurement)
            if n > 0:
                weight = weight**(1.0/n) # Normalize weight by number of landmarks.
            weights.append(weight)
        return weights

    def resample(self, weights):
        """Return a list of particles which have been resampled, proportional
           to the given weights."""
        particles = self.particles
        N = len(particles)
        normalized_weights = weights / np.sum(weights)
        cumulative_weights = np.cumsum(normalized_weights)
        # Draw N random samples between 0 and 1
        random_values = np.random.rand(N)

        # Resample particles
        new_particles = []
        for r in random_values:
            index = np.searchsorted(cumulative_weights, r)
            new_particles.append(particles[index])
        return new_particles

    def correct(self, cylinders, landmarks):
        """The correction step of the particle filter."""
        # First compute all weights.
        weights = self.compute_weights(cylinders, landmarks)
        # Then resample, based on the weight array.
        self.particles = self.resample(weights)

    def print_particles(self, file_desc):
        """Prints particles to given file_desc output."""
        if not self.particles:
            return
        file_desc.write("PA")
        for p in self.particles:
            line = " " + str(p[0]) + " " + str(p[1]) + " " + str(p[2])# + " "# + "\n" 
            file_desc.write(line)
        file_desc.write("\n")


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 50
    measured_state = (1850.0, 1897.0, 213.0 / 180.0 * pi)
    standard_deviations = (100.0, 100.0, 10.0 / 180.0 * pi)
    initial_particles = []
    for i in range(number_of_particles):
        initial_particles.append(tuple([
            random.gauss(measured_state[j], standard_deviations[j])
            for j in range(3)]))

    # Setup filter.
    pf = ParticleFilter(initial_particles,
                        robot_width, scanner_displacement,
                        control_motion_factor, control_turn_factor,
                        measurement_distance_stddev,
                        measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records.
    # This is the particle filter loop, with prediction and correction.
    f = open("particle_filter_corrected.txt", "w")
    for i in range(len(logfile.motor_ticks)):
        # Prediction.
        control = map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i])
        pf.predict(control)

        # Correction.
        cylinders = get_cylinders_from_scan(logfile.scan_data[i], depth_jump,
            minimum_valid_distance, cylinder_offset)
        pf.correct(cylinders, reference_cylinders)

        # Output particles.
        pf.print_particles(f)

    f.close()
