"""
GraphSLAM class for simple 2D graph-based SLAM testing.

This version supports adding fixed 2D poses and relative pose constraints,
computing residuals, and preparing for Gauss-Newton optimization.
"""

import numpy as np
from numpy import pi

def v2t(pose_vec):
    """Convert [x, y, theta] → 3×3 homogeneous transform."""
    x, y, theta = pose_vec
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    T = np.array([
        [cos_theta, -sin_theta, x],
        [sin_theta,  cos_theta, y],
        [0,          0,         1]
    ])
    return T

def t2v(T):
    """Convert 3×3 homogeneous transform → [x, y, theta]."""
    x = T[0, 2]
    y = T[1, 2]
    theta = np.arctan2(T[1, 0], T[0, 0])
    return np.array([x, y, theta])

class GraphSLAM:
    def __init__(self):
        self.poses = []               # List of 2D poses: [x, y, theta]
        self.constraints = []         # List of tuples: (i, j, z_ij, Omega_ij)

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
        Ti = v2t(xi)
        Tj = v2t(xj)

        Tij_pred = np.linalg.inv(Ti) @ Tj
        Zij_hat = t2v(Tij_pred)

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

def test_jacobians():
    """
    Unit test: Compare analytical and numerical Jacobians.
    """

    eps = 1e-6  # small perturbation for finite difference

    # Example poses (arbitrary but reasonable values)
    xi = np.array([1.0, 2.0, 30 * np.pi / 180])  # Pose i: x, y, theta
    xj = np.array([2.5, 3.0, 40 * np.pi / 180])  # Pose j: x, y, theta

    # Fake observation (not used for Jacobian test)
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
        A_numeric[:, k] = (e_perturbed - e0) / eps
        gs.poses[0] = xi  # restore

    # Numerical Jacobian for xj (B_ij)
    B_numeric = np.zeros((3, 3))
    for k in range(3):
        xj_perturbed = xj.copy()
        xj_perturbed[k] += eps
        gs.poses[1] = xj_perturbed  # update pose j
        e_perturbed = gs.compute_error(constraint)
        B_numeric[:, k] = (e_perturbed - e0) / eps
        gs.poses[1] = xj  # restore

        # Flip numerical Jacobians for sign convention
        A_numeric_flipped = -A_numeric
        B_numeric_flipped = -B_numeric

        # Compare results after flipping
        print("Analytical A_ij:")
        print(A_analytic)
        print("Numerical A_ij (flipped):")
        print(A_numeric_flipped)
        print("Difference A:")
        print(A_analytic - A_numeric_flipped)
        print()

        print("Analytical B_ij:")
        print(B_analytic)
        print("Numerical B_ij (flipped):")
        print(B_numeric_flipped)
        print("Difference B:")
        print(B_analytic - B_numeric_flipped)





if __name__ == '__main__':
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
    print("\n*****GRAPH SLAM OBJECT TEST******")
    gs.print_summary()

    # Expected Output:
    #     Poses:
    #   x0 = [0.0 0.0 0.0]
    #   x1 = [1.0 0.0 0.0]
    #   x2 = [2.0 0.0 0.0]
    # Constraints:
    #   x0 → x1 : [ 0.9  0.1  5.0 ]
    #   x1 → x2 : [ 1.1 -0.1  0.0 ]
    # Error:
    #   x1 → x2 : [-0.1  0.1  5.0 ]
    #   x1 → x2 : [ 0.1 -0.1  0.0 ]
    print("\n\n***********JACOBIAN TEST****************")
    test_jacobians()