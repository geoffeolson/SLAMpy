# The full Kalman filter, consisting of prediction and correction step.
#
# slam_07_f_kalman_filter
# You Tube SLAM D17
# Claus Brenner, 12.12.2012
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *
from slam_d_library import get_observations, write_cylinders
import os
os.chdir("Unit_D")


class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

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

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):
        # --->>> Put your method from 07_d_kalman_predict here.
        theta = state[2]
        l, r = control
        if r != l:
            # This is for the case r != l.
            # g has 3 components and the state has 3 components, so the
            # derivative of g with respect to all state variables is a
            # 3x3 matrix. To construct such a matrix in Python/Numpy,
            # use: m = array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
            # where 1, 2, 3 are the values of the first row of the matrix.
            # Don't forget to return this matrix.
            alpha = (r - l) / w
            rad = l/alpha
            dg1_dtheta = (rad + w/2.)*(cos(theta+alpha) - cos(theta))
            dg2_dtheta = (rad + w/2.)*(sin(theta+alpha) - sin(theta))

        else:
            # This is for the special case r == l.
            dg1_dtheta = -l*sin(theta)
            dg2_dtheta =  l*cos(theta)

        # The derivative of g with respect to x, y and theta is a 3x3 matrix.
        m = array([
            [1, 0, dg1_dtheta], 
            [0, 1, dg2_dtheta], 
            [0, 0, 1]])

        return m

    @staticmethod
    def dg_dcontrol(state, control, w):
        # --->>> Put your method from 07_d_kalman_predict here.
        x, y, theta = state
        l, r = tuple(control)

        if r != l:
            # --->>> Put your code here.
            # This is for the case l != r.
            # Note g has 3 components and control has 2, so the result
            # will be a 3x2 (rows x columns) matrix.
            alpha = (r - l) / w
            theta2 = theta + alpha
            dg1_dl =  w*r/((r-l)**2) * ( sin(theta2)-sin(theta)) - (r+l)/(2*(r-l)) * cos(theta2)
            dg2_dl =  w*r/((r-l)**2) * (-cos(theta2)+cos(theta)) - (r+l)/(2*(r-l)) * sin(theta2)
            dg3_dl = -1/w
            dg1_dr = -w*l/((r-l)**2) * ( sin(theta2)-sin(theta)) + (r+l)/(2*(r-l)) * cos(theta2)
            dg2_dr = -w*l/((r-l)**2) * (-cos(theta2)+cos(theta)) + (r+l)/(2*(r-l)) * sin(theta2)
            dg3_dr = 1/w
            
        else:
            dg1_dl = 0.5 * (cos(theta) + (l/w) * sin(theta))
            dg2_dl = 0.5 * (sin(theta) - (l/w) * cos(theta))
            dg3_dl = -1/w
            dg1_dr = 0.5 * (cos(theta) - (l/w) * sin(theta))
            dg2_dr = 0.5 * (sin(theta) + (l/w) * cos(theta))
            dg3_dr = 1/w           

        m = array([
            [dg1_dl, dg1_dr], 
            [dg2_dl, dg2_dr], 
            [dg3_dl, dg3_dr]])
            
        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):
        # --->>> Put your method from 07_d_kalman_predict here.
        """The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.
        left, right = control

        # --->>> Put your code to compute the new self.covariance here.
        # First, construct the control_covariance, which is a diagonal matrix.
        # In Python/Numpy, you may use diag([a, b]) to get
        # [[ a, 0 ],
        #  [ 0, b ]].
        # Then, compute G using dg_dstate and V using dg_dcontrol.
        # Then, compute the new self.covariance.
        # Note that the transpose of a Numpy array G is expressed as G.T,
        # and the matrix product of A and B is written as dot(A, B).
        # Writing A*B instead will give you the element-wise product, which
        # is not intended here.
        a1 = self.control_motion_factor
        a2 = self.control_turn_factor
        Gl2 = (a1 * left)**2 + (a2 * (left - right))**2
        Gr2 = (a1 * right)**2 + (a2 * (left - right))**2
        sigma_control = diag([Gl2, Gr2])
        Vt = self.dg_dcontrol(self.state, control, self.robot_width)
        Rt = dot(Vt, dot(sigma_control, Vt.T))
        Gt = self.dg_dstate(self.state, control, self.robot_width)
        self.covariance = dot(Gt, dot(self.covariance, Gt.T)) + Rt

        # state' = g(state, control)
        self.state = self.g(self.state, control, self.robot_width)

    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):
        # --->>> Put your method from 07_e_measurement derivative here.
        x, y, theta = state
        xm, ym = landmark
        d = scanner_displacement
        # --->>> Insert your code here.
        # Note that:
        # x y theta is state[0] state[1] state[2]
        # x_m y_m is landmark[0] landmark[1]
        # The Jacobian of h is a 2x3 matrix.
        xl = x + d * cos(theta)
        yl = y + d * sin(theta)
        dx = (xm - xl)
        dy = (ym - yl)
        q = dx**2 + dy**2
        dr_dx = - dx / sqrt(q)
        dr_dy = - dy / sqrt(q)
        dr_dtheta = d / sqrt(q) * (dx * sin(theta) - dy * cos(theta))
        da_dx = dy / q
        da_dy = - dx / q
        da_dtheta = - d / q * (dx * cos(theta) + dy * sin(theta)) - 1
        m = array([
            [dr_dx, dr_dy, dr_dtheta], 
            [da_dx, da_dy, da_dtheta]])

        return m

    def correct(self, measurement, landmark):
        """The correction step of the Kalman filter."""
        # You will have to compute:
        # H, using dh_dstate(...).
        # Q, a diagonal matrix, from self.measurement_distance_stddev and
        #  self.measurement_angle_stddev (remember: Q contains variances).
        # K, from self.covariance, H, and Q.
        #  Use linalg.inv(...) to compute the inverse of a matrix.
        # The innovation: it is easy to make an error here, because the
        #  predicted measurement and the actual measurement of theta may have
        #  an offset of +/- 2 pi. So here is a suggestion:
        #   innovation = array(measurement) -\
        #                self.h(self.state, landmark, self.scanner_displacement)
        #   innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        # Then, you'll have to compute the new self.state.
        # And finally, compute the new self.covariance. Use eye(3) to get a 3x3
        #  identity matrix.
        #
        # Hints:
        # dot(A, B) is the 'normal' matrix product (do not use: A*B).
        # A.T is the transposed of a matrix A (A itself is not modified).
        # linalg.inv(A) returns the inverse of A (A itself is not modified).
        # eye(3) returns a 3x3 identity matrix.
        H = self.dh_dstate(self.state, landmark, self.scanner_displacement)
        sigma_r = self.measurement_distance_stddev
        sigma_a = self.measurement_angle_stddev
        Q = diag([sigma_r**2, sigma_a**2])
        S = self.covariance
        K = S @ H.T @ linalg.inv(H @ S @ H.T + Q)
        innovation = array(measurement) - self.h(self.state, landmark, self.scanner_displacement)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        self.state = self.state + K @ innovation
        self.covariance = (eye(3) - K @ H) @ S

if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0
    max_cylinder_distance = 300.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Measured start position.
    initial_state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
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

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the Kalman filter loop, with prediction and correction.
    states = []
    covariances = []
    matched_ref_cylinders = []
    for i in range(len(logfile.motor_ticks)):
        # Prediction.
        control = array(logfile.motor_ticks[i]) * ticks_to_mm
        kf.predict(control)

        # Correction.
        observations = get_observations(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset,
            kf.state, scanner_displacement,
            reference_cylinders, max_cylinder_distance)
        for j in range(len(observations)):
            kf.correct(*observations[j])

        # Log state, covariance, and matched cylinders for later output.
        states.append(kf.state)
        covariances.append(kf.covariance)
        matched_ref_cylinders.append([m[1] for m in observations])

    # Write all states, all state covariances, and matched cylinders to file.
    with open("kalman_prediction_and_correction.txt", "w") as f:
        for i in range(len(states)):
            # Output the center of the scanner, not the center of the robot.
            # print >> f, "F %f %f %f" % \
            x = tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                                   scanner_displacement * sin(states[i][2]),
                                   0.0])
            line = "F " + str(x[0]) + " " + str(x[1]) + " " + str(x[2]) + "\n" 
            f.write(line)
            e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
            q = (e + (sqrt(covariances[i][2,2]),))
            line = "E " + str(q[0]) + " " + str(q[1]) + " " + str(q[2]) + " " + str(q[3]) + "\n" 
            f.write(line)
            write_cylinders(f, "W C", matched_ref_cylinders[i])

### Origional #################################################################
    # f = open("kalman_prediction_and_correction.txt", "w")
    # for i in range(len(states)):
    #     # Output the center of the scanner, not the center of the robot.
    #     print >> f, "F %f %f %f" % \
    #         tuple(states[i] + [scanner_displacement * cos(states[i][2]),
    #                            scanner_displacement * sin(states[i][2]),
    #                            0.0])
    #     # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
    #     e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
    #     print >> f, "E %f %f %f %f" % (e + (sqrt(covariances[i][2,2]),))
    #     # Also, write matched cylinders.
    #     write_cylinders(f, "W C", matched_ref_cylinders[i])        

    # f.close()


### Example #####################################################################################

        # Write all states, all state covariances, and matched cylinders to file.
    # f = open("kalman_prediction.txt", "w")
    # for i in range(len(states)):
    #     # Output the center of the scanner, not the center of the robot.
    #     print >> f, "F %f %f %f" % \
    #         tuple(states[i] + [scanner_displacement * cos(states[i][2]),
    #                            scanner_displacement * sin(states[i][2]),
    #                            0.0])
    #     # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
    #     e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
    #     print >> f, "E %f %f %f %f" % (e + (sqrt(covariances[i][2,2]),))
    #     #(angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))
    # f.close()

    # with open("kalman_prediction.txt", "w") as f:
    #     for i in range(len(states)):
    #         # Output the center of the scanner, not the center of the robot.
    #         #print >> f, "F %f %f %f" % \
    #         x = tuple(states[i] + [scanner_displacement * cos(states[i][2]),
    #                         scanner_displacement * sin(states[i][2]),
    #                         0.0])
    #         line = "F " + str(x[0]) + " " + str(x[1]) + " " + str(x[2]) + "\n" 
    #         f.write(line)
    #         e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
    #         q = (e + (sqrt(covariances[i][2,2]),))
    #         line = "E " + str(q[0]) + " " + str(q[1]) + " " + str(q[2]) + " " + str(q[3]) + "\n" 
    #         f.write(line)
