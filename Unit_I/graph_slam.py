"""
GraphSLAM class for simple 2D graph-based SLAM testing.

This version supports adding fixed 2D poses and relative pose constraints,
computing residuals, and preparing for Gauss-Newton optimization.
"""
from scipy.stats import sigmaclip
from lego_robot import *
import numpy as np
from numpy import pi, sin, cos, pi, atan2
import numpy as np
import os

# def v2t(pose_vec):
#     """Convert [x, y, theta] → 3×3 homogeneous transform. """
#     x, y, theta = pose_vec
#     cos_theta = np.cos(theta)
#     sin_theta = np.sin(theta)
#     T = np.array([
#         [cos_theta, -sin_theta, x],
#         [sin_theta,  cos_theta, y],
#         [0,          0,         1]
#     ])
#     return T

# def t2v(T):
#     """Convert 3×3 homogeneous transform → [x, y, theta]."""
#     x = T[0, 2]
#     y = T[1, 2]
#     theta = np.arctan2(T[1, 0], T[0, 0])
#     return np.array([x, y, theta])


# class ExtendedKalmanFilter:
#     def __init__(self, state, covariance,
#                  robot_width,
#                  control_motion_factor, control_turn_factor):
#         # The state. This is the core data of the Kalman filter.
#         self.state = state
#         self.covariance = covariance

#         #>>>>>>>>>>>>>>UNTESTED>>>>>>>>>>>>>>>.
#         self.last_relative_motion = np.zeros(3)
#         self.last_motion_covariance = None
#         # <<<<<<<<<<<<<<END UNTESTED<<<<<<<<<<<<<<

#         # Some constants.
#         self.robot_width = robot_width
#         self.control_motion_factor = control_motion_factor
#         self.control_turn_factor = control_turn_factor

#     @staticmethod
#     def g(state, control, w):
#         x, y, theta = state
#         l, r = control
#         if r != l:
#             alpha = (r - l) / w
#             rad = l/alpha
#             g1 = x + (rad + w/2.)*( sin(theta+alpha) - sin(theta))
#             g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
#             g3 = (theta + alpha + pi) % (2*pi) - pi
#         else:
#             g1 = x + l * cos(theta)
#             g2 = y + l * sin(theta)
#             g3 = theta

#         return np.array([g1, g2, g3])

#     @staticmethod
#     def dg_dstate(state, control, w):
#         theta = state[2]
#         l, r = control
#         if r != l:
#             # This is for the case r != l.
#             # g has 3 components and the state has 3 components, so the
#             # derivative of g with respect to all state variables is a
#             # 3x3 matrix. To construct such a matrix in Python/Numpy,
#             # use: m = array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
#             # where 1, 2, 3 are the values of the first row of the matrix.
#             # Don't forget to return this matrix.
#             alpha = (r - l) / w
#             rad = l/alpha
#             dg1_dtheta = (rad + w/2.)*(cos(theta+alpha) - cos(theta))
#             dg2_dtheta = (rad + w/2.)*(sin(theta+alpha) - sin(theta))

#         else:
#             # This is for the special case r == l.
#             dg1_dtheta = -l*sin(theta)
#             dg2_dtheta =  l*cos(theta)

#         # The derivative of g with respect to x, y and theta is a 3x3 matrix.
#         m = np.array([
#             [1, 0, dg1_dtheta], 
#             [0, 1, dg2_dtheta], 
#             [0, 0, 1]])

#         return m

#     @staticmethod
#     def dg_dcontrol(state, control, w):
#         x, y, theta = state
#         l, r = tuple(control)

#         if r != l:
#             # --->>> Put your code here.
#             # This is for the case l != r.
#             # Note g has 3 components and control has 2, so the result
#             # will be a 3x2 (rows x columns) matrix.
#             alpha = (r - l) / w
#             theta2 = theta + alpha
#             dg1_dl =  w*r/((r-l)**2) * ( sin(theta2)-sin(theta)) - (r+l)/(2*(r-l)) * cos(theta2)
#             dg2_dl =  w*r/((r-l)**2) * (-cos(theta2)+cos(theta)) - (r+l)/(2*(r-l)) * sin(theta2)
#             dg3_dl = -1/w
#             dg1_dr = -w*l/((r-l)**2) * ( sin(theta2)-sin(theta)) + (r+l)/(2*(r-l)) * cos(theta2)
#             dg2_dr = -w*l/((r-l)**2) * (-cos(theta2)+cos(theta)) + (r+l)/(2*(r-l)) * sin(theta2)
#             dg3_dr = 1/w
            
#         else:
#             dg1_dl = 0.5 * (cos(theta) + (l/w) * sin(theta))
#             dg2_dl = 0.5 * (sin(theta) - (l/w) * cos(theta))
#             dg3_dl = -1/w
#             dg1_dr = 0.5 * (cos(theta) - (l/w) * sin(theta))
#             dg2_dr = 0.5 * (sin(theta) + (l/w) * cos(theta))
#             dg3_dr = 1/w           

#         m = np.array([
#             [dg1_dl, dg1_dr], 
#             [dg2_dl, dg2_dr], 
#             [dg3_dl, dg3_dr]])
            
#         return m

#     @staticmethod
#     def get_error_ellipse(covariance):
#         """Return the position covariance (which is the upper 2x2 submatrix)
#            as a triple: (main_axis_angle, stddev_1, stddev_2), where
#            main_axis_angle is the angle (pointing direction) of the main axis,
#            along which the standard deviation is stddev_1, and stddev_2 is the
#            standard deviation along the other (orthogonal) axis."""
#         eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
#         angle = atan2(eigenvects[1,0], eigenvects[0,0])
#         return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

#     def predict_old(self, control):
#         """The prediction step of the Kalman filter."""
#         # covariance' = G * covariance * GT + R
#         # where R = V * (covariance in control space) * VT.
#         # Covariance in control space depends on move distance.
#         left, right = control
#         a1 = self.control_motion_factor
#         a2 = self.control_turn_factor
#         Gl2 = (a1 * left)**2 + (a2 * (left - right))**2
#         Gr2 = (a1 * right)**2 + (a2 * (left - right))**2
#         sigma_control = np.diag([Gl2, Gr2])
#         Vt = self.dg_dcontrol(self.state, control, self.robot_width)
#         Rt = np.dot(Vt, np.dot(sigma_control, Vt.T))
#         Gt = self.dg_dstate(self.state, control, self.robot_width)
#         self.covariance = np.dot(Gt, np.dot(self.covariance, Gt.T)) + Rt

#         # state' = g(state, control)
#         self.state = self.g(self.state, control, self.robot_width)

#     #>>>>>>>>>>>>>>UNTESTED>>>>>>>>>>>>>>>.
#     def get_last_relative_motion(self):
#         return self.last_relative_motion

#     def get_last_motion_covariance(self):
#         return self.last_motion_covariance
#     #<<<<<<<<<<<<<<END UNTESTED<<<<<<<<<<<<<<

#     def predict(self, control):
#         """The prediction step of the Kalman filter."""

#         # Store previous state for relative motion
#         x_prev = self.state.copy()

#         # 1. Predict covariance
#         left, right = control
#         a1 = self.control_motion_factor
#         a2 = self.control_turn_factor
#         Gl2 = (a1 * left)**2 + (a2 * (left - right))**2
#         Gr2 = (a1 * right)**2 + (a2 * (left - right))**2
#         sigma_control = np.diag([Gl2, Gr2])

#         Vt = self.dg_dcontrol(self.state, control, self.robot_width)
        
    
    
#         #>>>>>>>>>>>>>>UNTESTED>>>>>>>>>>>>>>>.
#         Rt = np.dot(Vt, np.dot(sigma_control, Vt.T))
#         self.last_motion_covariance = Rt
#         #<<<<<<<<<<<<<<END UNTESTED<<<<<<<<<<<<<<

#         Gt = self.dg_dstate(self.state, control, self.robot_width)
#         self.covariance = np.dot(Gt, np.dot(self.covariance, Gt.T)) + Rt

#         # 2. Predict new state
#         x_curr = self.g(self.state, control, self.robot_width)
#         self.state = x_curr

#         # 3. Compute relative motion in the local frame of x_prev
#         dx = x_curr[0] - x_prev[0]
#         dy = x_curr[1] - x_prev[1]
#         dtheta = x_curr[2] - x_prev[2]
#         theta = x_prev[2]

#         # Rotate into local frame of x_prev
#         rot_dx = cos(-theta) * dx - sin(-theta) * dy
#         rot_dy = sin(-theta) * dx + cos(-theta) * dy
#         rot_dtheta = (dtheta + pi) % (2 * pi) - pi

        
#         #>>>>>>>>>>>>>>UNTESTED>>>>>>>>>>>>>>>.
#         # 4. Store the result
#         self.last_relative_motion = np.array([rot_dx, rot_dy, rot_dtheta])
#         #<<<<<<<<<<<<<<END UNTESTED<<<<<<<<<<<<<<

class GraphSLAM:
    def __init__(self):
        self.poses = []               # List of 2D poses: [x, y, theta]
        self.constraints = []         # List of tuples: (i, j, z_ij, Omega_ij)
    
    @staticmethod
    def v2t(pose_vec):
        """Convert [x, y, theta] → 3×3 homogeneous transform. """
        x, y, theta = pose_vec
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        T = np.array([
            [cos_theta, -sin_theta, x],
            [sin_theta,  cos_theta, y],
            [0,          0,         1]
        ])
        return T

    @staticmethod
    def t2v(T):
        """Convert 3×3 homogeneous transform → [x, y, theta]."""
        x = T[0, 2]
        y = T[1, 2]
        theta = np.arctan2(T[1, 0], T[0, 0])
        return np.array([x, y, theta])

    def add_pose(self, pose):
        """Add a new pose to the graph."""
        self.poses.append(np.array(pose))

    def add_constraint(self, i, j, z_ij, Omega_ij):
        """
        Add a relative pose constraint between pose i and pose j.
        z_ij: expected relative motion from i to j
        Omega_ij: 3x3 information matrix
        """
        self.constraints.append((i, j, np.array(z_ij), np.array(Omega_ij)))

    def compute_error(self, constraint):
        """
        Compute the residual e_ij = z_ij - z_hat_ij
        where z_hat_ij is the predicted relative pose from i to j
        """
        i, j, Zij, _ = constraint
        xi = self.poses[i]
        xj = self.poses[j]

        # Convert to transforms
        Ti = GraphSLAM.v2t(xi)
        Tj = GraphSLAM.v2t(xj)

        Tij_pred = np.linalg.inv(Ti) @ Tj
        Zij_hat = GraphSLAM.t2v(Tij_pred)

        error = Zij - Zij_hat
        error[2] = (error[2] + np.pi) % (2 * np.pi) - np.pi
        return error

    def compute_jacobians(xi, xj):
        """
        Compute Jacobians A_ij and B_ij for the relative pose constraint between xi and xj.

        Parameters:
        xi : ndarray (3,) - pose i [x_i, y_i, theta_i]
        xj : ndarray (3,) - pose j [x_j, y_j, theta_j]

        Returns:
        A_ij : ndarray (3, 3)
        B_ij : ndarray (3, 3)
        """
        # Extract values
        xi_x, xi_y, xi_theta = xi
        xj_x, xj_y, xj_theta = xj

        # Compute delta t
        delta_t = np.array([xj_x - xi_x, xj_y - xi_y])

        # Rotation matrix Ri and its transpose
        cos_theta = np.cos(xi_theta)
        sin_theta = np.sin(xi_theta)
        Ri = np.array([[cos_theta, -sin_theta],
                        [sin_theta,  cos_theta]])
        Ri_T = Ri.T

        # Skew-symmetric matrix S
        S = np.array([[0, -1],
                        [1,  0]])

        # Compute blocks for A_ij
        A_pos = -Ri_T
        A_theta = Ri_T @ S @ delta_t
        A_ij = np.zeros((3, 3))
        A_ij[0:2, 0:2] = A_pos
        A_ij[0:2, 2] = A_theta
        A_ij[2, 2] = -1

        # Compute blocks for B_ij
        B_ij = np.zeros((3, 3))
        B_ij[0:2, 0:2] = Ri_T
        B_ij[2, 2] = 1

        return A_ij, B_ij

    def build_linear_system(self):
        """
        Build the linear system H and b for Graph-Based SLAM.
        Follows directly from the derivation in the README.md
        """

        N = len(self.poses)
        H = np.zeros((3 * N, 3 * N))
        b = np.zeros((3 * N, ))

        for constraint in self.constraints:
            i, j, z_ij, Omega_ij = constraint

            # Get current poses
            xi = self.poses[i]
            xj = self.poses[j]

            # Compute error and Jacobians
            e_ij = self.compute_error(constraint)
            A_ij, B_ij = GraphSLAM.compute_jacobians(xi, xj)

            # Compute blocks for H
            H_ii = A_ij.T @ Omega_ij @ A_ij
            H_ij = A_ij.T @ Omega_ij @ B_ij
            H_ji = B_ij.T @ Omega_ij @ A_ij
            H_jj = B_ij.T @ Omega_ij @ B_ij

            # Compute blocks for b
            b_i = - A_ij.T @ Omega_ij @ e_ij
            b_j = - B_ij.T @ Omega_ij @ e_ij

            # Assemble into global H and b
            i_idx = slice(3 * i, 3 * i + 3)
            j_idx = slice(3 * j, 3 * j + 3)

            H[i_idx, i_idx] += H_ii
            H[i_idx, j_idx] += H_ij
            H[j_idx, i_idx] += H_ji
            H[j_idx, j_idx] += H_jj

            b[i_idx] += b_i
            b[j_idx] += b_j

        return H, b

    def solve(self, max_iterations=10, tol=1e-4):
        """
        Gauss-Newton optimizer for Graph-Based SLAM.
        Follows the full system described in README.md.
        """
        N = len(self.poses)

        for iteration in range(max_iterations):
            print(f"\nIteration {iteration+1}")

            # Build linear system
            H, b = self.build_linear_system()

            # Apply gauge fixing (anchor first pose)
            H[0:3, :] = 0
            H[:, 0:3] = 0
            H[0:3, 0:3] = np.eye(3)
            b[0:3] = 0

            # Solve the system: H Δx = -b
            delta_x = np.linalg.solve(H, -b)

            # Apply updates to poses
            for i in range(N):
                idx = slice(3 * i, 3 * i + 3)
                dx_i = delta_x[idx]
                self.poses[i] += dx_i

            # Check for convergence
            norm_dx = np.linalg.norm(delta_x)
            print(f"  Norm of delta_x: {norm_dx:.6f}")
            if norm_dx < tol:
                print("Converged.")
                break


    def print_summary(self):
        print("Poses:")
        for idx, p in enumerate(self.poses):
            print(f"  x{idx} = {p}")
        print("Constraints:")
        for i, j, z, _ in self.constraints:
            Zij = np.array([z[0],z[1],z[2] * 180 / pi])
            print(f"  x{i} → x{j} : {Zij}")
        print("Error:")
        for constraint in self.constraints:
            i, j, _, _ = constraint
            e_ij = self.compute_error(constraint)
            e_ij[2] = e_ij[2] * 180 / pi
            print(f"  x{i} → x{j} : {e_ij}")

def test_object_creation():
    """
    Unit test: Create Object and Test.
     Expected Output:
         Poses:
       x0 = [0.0 0.0 0.0]
       x1 = [1.0 0.0 0.0]
       x2 = [2.0 0.0 0.0]
     Constraints:
       x0 → x1 : [ 0.9  0.1  5.0 ]
       x1 → x2 : [ 1.1 -0.1  0.0 ]
     Error:
       x1 → x2 : [-0.1  0.1  5.0 ]
       x1 → x2 : [ 0.1 -0.1  0.0 ]
    """
    # create Graph Slam object
    gs = GraphSLAM()

    #Add Poses
    x0 = [0.0, 0.0, 0.0]   # origin
    x1 = [1.0, 0.0, 0.0]   # 1 meter forward
    x2 = [2.0, 0.0, 0.0]   # 2 meter forward

    gs.add_pose(x0)
    gs.add_pose(x1)
    gs.add_pose(x2)

    #Add constraint
    z_ij = [0.9, 0.1, 5 * pi / 180]
    Omega_ij = np.diag([1.0, 1.0, 1.0])
    gs.add_constraint(0, 1, z_ij, Omega_ij)

    z_ij = [1.1, -0.1, 0.0]
    Omega_ij = np.diag([1.0, 1.0, 1.0])
    gs.add_constraint(1, 2, z_ij, Omega_ij)

    #print Results
    gs.print_summary()

    return gs

def test_jacobians():
    """
    Unit test: Compare analytical and numerical Jacobians.
    """

    eps = 1e-6  # small perturbation for finite difference

    # Example poses (arbitrary but reasonable values)
    xi = np.array([1.0, 2.0, 30 * np.pi / 180])  # Pose i: x, y, theta
    xj = np.array([2.5, 3.0, 40 * np.pi / 180])  # Pose j: x, y, theta

    # Dummy observation and information matrix (not used for Jacobian test)
    z_ij = np.array([0.0, 0.0, 0.0])
    Omega_ij = np.eye(3)

    # Build artificial constraint to reuse existing error function
    gs = GraphSLAM()
    gs.add_pose(xi)
    gs.add_pose(xj)
    constraint = (0, 1, z_ij, Omega_ij)

    # Compute baseline error
    e0 = gs.compute_error(constraint)

    # Compute analytical Jacobians
    A_analytic, B_analytic = GraphSLAM.compute_jacobians(xi, xj)

    # Numerical Jacobian for xi (A_ij)
    A_numeric = np.zeros((3, 3))
    for k in range(3):
        xi_perturbed = xi.copy()
        xi_perturbed[k] += eps
        gs.poses[0] = xi_perturbed  # update pose i
        e_perturbed = gs.compute_error(constraint)

        # IMPORTANT NOTE ON THE SIGN:
        # The error function is defined as: e_ij = z_ij - hat_z_ij
        # Perturbing xi affects hat_z_ij, and therefore affects e_ij negatively.
        # To correctly approximate the Jacobian, we must negate the finite difference:
        A_numeric[:, k] = -(e_perturbed - e0) / eps

        gs.poses[0] = xi  # restore

    # Numerical Jacobian for xj (B_ij)
    B_numeric = np.zeros((3, 3))
    for k in range(3):
        xj_perturbed = xj.copy()
        xj_perturbed[k] += eps
        gs.poses[1] = xj_perturbed  # update pose j
        e_perturbed = gs.compute_error(constraint)

        # Same sign logic applies to xj perturbation:
        B_numeric[:, k] = -(e_perturbed - e0) / eps

        gs.poses[1] = xj  # restore

    # Compare results
    print("\n\n***********JACOBIAN TEST****************")
    print("\n\n\n******A*********")
    print("Analytical A_ij:")
    print(A_analytic)
    print("Numerical A_ij:")
    print(A_numeric)
    print("Difference A:")
    print(A_analytic - A_numeric)

    print("\n\n\n******B*********")
    print("Analytical B_ij:")
    print(B_analytic)
    print("Numerical B_ij:")
    print(B_numeric)
    print("Difference B:")
    print(B_analytic - B_numeric)

def test_linear_system():
    """
    Unit test: Verify correct construction of H and b.
    This test simply runs the function and prints the outputs for inspection.
    """

    # Build Graph SLAM object
    gs = test_object_creation()

    # Build system
    H, b = gs.build_linear_system()

    # Print system matrices for inspection
    print("\n\n*********** LINEAR SYSTEM TEST ***********")
    print("Information matrix H:")
    print(H)
    print("\nRight-hand side vector b:")
    print(b)

def test_solver():
    """
    Unit test: Verify full Gauss-Newton optimization converges correctly.
    """

    print("\n\n*********** SOLVER TEST ***********")

    # Build Graph SLAM object using existing helper
    
    print("\nInitial State:")
    gs = test_object_creation()

    # Print initial error
    print("\nInitial state:")
    gs.print_summary()

    # Run optimization
    gs.solve(max_iterations=10, tol=1e-6)

    # Print final optimized state
    print("\nFinal optimized state:")
    gs.print_summary()

def test_prediction_old():
    print("\n*****GRAPH SLAM PREDICTION TEST******")
    initial_state = np.array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    initial_covariance = np.diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    ticks_to_mm = 0.349
    robot_width = 155.0
    motion_factor = 0.35
    turn_factor = 0.6

    ekf = ExtendedKalmanFilter(
        initial_state,
        initial_covariance,
        robot_width,
        motion_factor,
        turn_factor
    )

    # Load odometry
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Output file
    with open("graph_slam_prediction.txt", "w") as f:
        for ticks in logfile.motor_ticks:
            control = np.array(ticks) * ticks_to_mm
            ekf.predict(control)
            x, y, θ = ekf.state
            line = f"F {x} {y} {θ}\n"
            f.write(line)

def test_prediction():
    print("\n*****GRAPH SLAM PREDICTION TEST******")
    
    # EKF setup (front-end)
    initial_state = np.array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    initial_covariance = np.diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    ticks_to_mm = 0.349
    robot_width = 155.0
    motion_factor = 0.35
    turn_factor = 0.6

    ekf = ExtendedKalmanFilter(
        initial_state,
        initial_covariance,
        robot_width,
        motion_factor,
        turn_factor
    )

    #>>>>>>>>>>>>>UNTESTED>>>>>>>>>>>>>>>>>>>>>>>
    # Graph SLAM setup (back-end)
    graph_slam = GraphSLAM()
    graph_slam.add_pose(initial_state.copy())  # Pose x0
   #<<<<<<<<<<<<<<END UNTESTED<<<<<<<<<<<<<<

    # Load odometry
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    #>>>>>>>>>>>>UNTESTED>>>>>>>>>>>>>>>>>>>
    # Main loop
    for i, ticks in enumerate(logfile.motor_ticks):
        control = np.array(ticks) * ticks_to_mm
        ekf.predict(control)

        # Add new pose
        graph_slam.add_pose(ekf.state.copy())

        # Add constraint from previous pose to current
        z_ij = ekf.get_last_relative_motion()
        Sigma_ij = ekf.get_last_motion_covariance()
        Omega_ij = np.linalg.inv(Sigma_ij)  # Information matrix
        graph_slam.add_constraint(i, i + 1, z_ij, Omega_ij)

    # Write result to file
    with open("graph_slam_prediction.txt", "w") as f:
        for x, y, θ in graph_slam.poses:
            f.write(f"F {x} {y} {θ}\n")

    print(f"Wrote {len(graph_slam.poses)} poses and constraints to graph_slam_prediction.txt")
    #<<<<<<<<<<<<<<<<<<END UNTESTED<<<<<<<<<<<<
    


if __name__ == '__main__':
    os.chdir("Unit_H")
    print("\n*****GRAPH SLAM OBJECT TEST******")
    test_object_creation()
    test_jacobians()
    test_linear_system()
    test_solver()
    #test_prediction()







    # print("\n***** FRAMEWORK TEST: READ ODOMETRY AND LOG POSES ******")
    
    # from lego_robot import LegoLogfile
    # from math import cos, sin
    # from numpy import array

    # scanner_displacement = 30.0  # mm
    # ticks_to_mm = 0.349
    # robot_width = 155.0

    # # Initial pose: (x, y, θ)
    # initial_pose = array([0.0, 0.0, 0.0])
    # poses = [tuple(initial_pose)]

    # # Load odometry using Brenner's class
    # logfile = LegoLogfile()
    # logfile.read("robot4_motors.txt")

    # # Naive forward motion model (no optimization)
    # x, y, theta = initial_pose
    # for ticks in logfile.motor_ticks:
    #     l_ticks, r_ticks = ticks
    #     l = l_ticks * ticks_to_mm
    #     r = r_ticks * ticks_to_mm
    #     if abs(l - r) < 1e-6:
    #         x += l * cos(theta)
    #         y += l * sin(theta)
    #     else:
    #         alpha = (r - l) / robot_width
    #         R = l / alpha
    #         cx = x - (R + robot_width / 2.0) * sin(theta)
    #         cy = y + (R + robot_width / 2.0) * cos(theta)
    #         theta += alpha
    #         x = cx + (R + robot_width / 2.0) * sin(theta)
    #         y = cy - (R + robot_width / 2.0) * cos(theta)
    #     poses.append((x, y, theta))

    # # Write result using Brenner-compatible format
    # with open("graph_slam_result.txt", "w") as f:
    #     for x, y, theta in poses:
    #         f.write(f"{x:.6f} {y:.6f} {theta:.6f}\n")

    # print("Wrote", len(poses), "poses to graph_slam_result.txt")





##################################################################
#     DEPRICATED CODE
##################################################################
    # # Create graph object and set up small problem
    # gs = GraphSLAM()

    # # Add poses (same as previous tests)
    # x0 = [0.0, 0.0, 0.0]
    # x1 = [1.0, 0.0, 0.0]
    # x2 = [2.0, 0.0, 0.0]

    # gs.add_pose(x0)
    # gs.add_pose(x1)
    # gs.add_pose(x2)

    # # Add constraints
    # z_ij = [0.9, 0.1, 5 * pi / 180]
    # Omega_ij = np.diag([1.0, 1.0, 1.0])
    # gs.add_constraint(0, 1, z_ij, Omega_ij)

    # z_ij = [1.1, -0.1, 0.0]
    # Omega_ij = np.diag([1.0, 1.0, 1.0])
    # gs.add_constraint(1, 2, z_ij, Omega_ij)



# def test_jacobians():
#     """
#     Unit test: Compare analytical and numerical Jacobians.
#     """

#     delta_x = 1e-6  # small perturbation for finite difference

#     # Example poses (arbitrary but reasonable values)
#     xi = np.array([1.0, 2.0, 30 * np.pi / 180])  # Pose i: x, y, theta
#     xj = np.array([2.5, 3.0, 40 * np.pi / 180])  # Pose j: x, y, theta

#     # Fake observation (not used for Jacobian test)
#     z_ij = np.array([0.0, 0.0, 0.0])
#     Omega_ij = np.eye(3)

#     # Build artificial constraint to reuse existing error function
#     gs = GraphSLAM()
#     gs.add_pose(xi)
#     gs.add_pose(xj)
#     constraint = (0, 1, z_ij, Omega_ij)

#     # Compute baseline error
#     e0 = gs.compute_error(constraint)

#     # Compute analytical Jacobians
#     A_analytic, B_analytic = GraphSLAM.compute_jacobians(xi, xj)

#     # Numerical Jacobian for xi (A_ij)
#     A_numeric = np.zeros((3, 3))
#     for k in range(3):
#         xi_perturbed = xi.copy()
#         xi_perturbed[k] += delta_x
#         gs.poses[0] = xi_perturbed  # update pose i
#         e_perturbed = gs.compute_error(constraint)
#         A_numeric[:, k] = (e_perturbed - e0) / delta_x
#         gs.poses[0] = xi  # restore

#     # Numerical Jacobian for xj (B_ij)
#     B_numeric = np.zeros((3, 3))
#     for k in range(3):
#         xj_perturbed = xj.copy()
#         xj_perturbed[k] += delta_x
#         gs.poses[1] = xj_perturbed  # update pose j
#         e_perturbed = gs.compute_error(constraint)
#         B_numeric[:, k] = (e_perturbed - e0) / delta_x
#         gs.poses[1] = xj  # restore

#         # Flip numerical Jacobians for sign convention
#         A_numeric_flipped = -A_numeric
#         B_numeric_flipped = -B_numeric

#         # Compare results after flipping
        
#     print("\n\n******A*********")
#     print("Analytical A_ij:")
#     print(A_analytic)
#     print("Numerical A_ij (flipped):")
#     print(A_numeric_flipped)
#     print("Difference A:")
#     print(A_analytic - A_numeric_flipped)
#     print()
#     print("\n\n******B*********")
#     print("Analytical B_ij:")
#     print(B_analytic)
#     print("Numerical B_ij (flipped):")
#     print(B_numeric_flipped)
#     print("Difference B:")
#     print(B_analytic - B_numeric_flipped)


    # # create Graph Slam object
    # gs = GraphSLAM()

    # #Add Poses
    # x0 = [0.0, 0.0, 0.0]   # origin
    # x1 = [1.0, 0.0, 0.0]   # 1 meter forward
    # x2 = [2.0, 0.0, 0.0]   # 2 meter forward

    # gs.add_pose(x0)
    # gs.add_pose(x1)
    # gs.add_pose(x2)

    # #Add constraint
    # z_ij = [0.9, 0.1, 5 * pi / 180]
    # Omega_ij = np.diag([1.0, 1.0, 1.0])
    # gs.add_constraint(0, 1, z_ij, Omega_ij)

    # z_ij = [1.1, -0.1, 0.0]
    # Omega_ij = np.diag([1.0, 1.0, 1.0])
    # gs.add_constraint(1, 2, z_ij, Omega_ij)

    # #print Results
    # gs.print_summary()


    # # Expected Output:
    # #     Poses:
    # #   x0 = [0.0 0.0 0.0]
    # #   x1 = [1.0 0.0 0.0]
    # #   x2 = [2.0 0.0 0.0]
    # # Constraints:
    # #   x0 → x1 : [ 0.9  0.1  5.0 ]
    # #   x1 → x2 : [ 1.1 -0.1  0.0 ]
    # # Error:
    # #   x1 → x2 : [-0.1  0.1  5.0 ]
    # #   x1 → x2 : [ 0.1 -0.1  0.0 ]