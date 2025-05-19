# The particle filter, prediciton and correction.
# In addition to the previous code:
# 1.
# the second moments are computed and are output as an error ellipse and
# heading variance.
# 2.
# the particles are initialized uniformly distributed in the arena, and a
# larger number of particles is used.
# 3.
# predict and correct are only called when control is nonzero.
#
# slam_08_d_density_error_ellipse.
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
    # --->>> Copy all the methods from the previous solution here.
    # These are methods from __init__() to get_mean().
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
            line = " " + str(p[0]) + " " + str(p[1]) + " " + str(p[2])
            file_desc.write(line)
        file_desc.write("\n")

    def get_mean(self):
        """Compute mean position and heading from all particles."""
        # --->>> This is the new code you'll have to implement.
        # Return a tuple: (mean_x, mean_y, mean_heading).
        # Compute mean x, y and heading from all particles.
        # headings = [p[2] for p in self.particles]
        # mean_heading = atan2(sum(sin(headings)), sum(cos(headings)))
        # Note that the mean heading is not the average of all headings.
        # It is computed by summing the x and y components of the unit
        # circle, and then taking the arctangent of the result.
        # This is necessary to avoid the problem of discontinuity at 0 and 2 pi.
        Px = 0.0
        Py = 0.0
        Hx = 0.0
        Hy = 0.0
        n = len(self.particles)
        for p in self.particles:
            x, y, heading = p
            Px += x
            Py += y
            Hx += cos(heading)
            Hy += sin(heading)
        x = Px / n
        y = Py / n
        heading = atan2(Hy, Hx)
        # Normalize mean heading to [-pi, pi] range.
        #heading = (heading + pi) % (2 * pi) - pi
        return (x, y, heading)

    # *** Modification 1: Extension: This computes the error ellipse.
    def get_error_ellipse_and_heading_variance(self, mean):
        """Returns a tuple: (angle, stddev1, stddev2, heading-stddev) which is
           the orientation of the xy error ellipse, the half axis 1, half axis 2,
           and the standard deviation of the heading."""
        center_x, center_y, center_heading = mean
        n = len(self.particles)
        if n < 2:
            return (0.0, 0.0, 0.0, 0.0)

        # Compute covariance matrix in xy.
        sxx, sxy, syy = 0.0, 0.0, 0.0
        for p in self.particles:
            dx = p[0] - center_x
            dy = p[1] - center_y
            sxx += dx * dx
            sxy += dx * dy
            syy += dy * dy
        cov_xy = np.array([[sxx, sxy], [sxy, syy]]) / (n-1)

        # Get variance of heading.
        var_heading = 0.0
        for p in self.particles:
            dh = (p[2] - center_heading + pi) % (2*pi) - pi
            var_heading += dh * dh
        var_heading = var_heading / (n-1)

        # Convert xy to error ellipse.
        eigenvals, eigenvects = np.linalg.eig(cov_xy)
        ellipse_angle = atan2(eigenvects[1,0], eigenvects[0,0])

        return (ellipse_angle, sqrt(abs(eigenvals[0])),
                sqrt(abs(eigenvals[1])),
                sqrt(var_heading))


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
    # *** Modification 2: Generate the particles uniformly distributed.
    # *** Also, use a large number of particles.
    number_of_particles = 500
    # Alternative: uniform init.
    initial_particles = []
    for i in range(number_of_particles):
        initial_particles.append((
            random.uniform(0.0, 2000.0), random.uniform(0.0, 2000.0),
            random.uniform(-pi, pi)))

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
    f = open("particle_filter_ellipse.txt", "w")
    for i in range(len(logfile.motor_ticks)):
        control = list(map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i]))
        # *** Modification 3: Call the predict/correct step only if there
        # *** is nonzero control.
        if control != [0.0, 0.0]:
            # Prediction.
            pf.predict(control)

            # Correction.
            cylinders = get_cylinders_from_scan(logfile.scan_data[i], depth_jump,
                minimum_valid_distance, cylinder_offset)
            pf.correct(cylinders, reference_cylinders)

        # Output particles.
        pf.print_particles(f)
        
        # Output state estimated from all particles.
        # mean = pf.get_mean()
        # print >> f, "F %.0f %.0f %.3f" %\
        #       (mean[0] + scanner_displacement * cos(mean[2]),
        #        mean[1] + scanner_displacement * sin(mean[2]),
        #        mean[2])

        # # Output error ellipse and standard deviation of heading.
        # errors = pf.get_error_ellipse_and_heading_variance(mean)
        # print >> f, "E %.3f %.0f %.0f %.3f" % errors

        m = pf.get_mean()
        x = m[0] + scanner_displacement * cos(m[2])
        y = m[1] + scanner_displacement * sin(m[2])
        heading = m[2]
        line = "F " + str(x) + " " + str(y) + " " + str(heading) + "\n"
        f.write(line)

        # # Output error ellipse and standard deviation of heading.
        # e = pf.get_error_ellipse_and_heading_variance(mean)
        # print >> f, "E %.3f %.0f %.0f %.3f" % errors

        
        # # Output error ellipse and standard deviation of heading.
        e = pf.get_error_ellipse_and_heading_variance(m)
        line = "E " + str(e[0]) + " " + str(e[1]) + " " + str(e[2]) + " " + str(e[3]) + "\n"
        f.write(line)

    f.close()
