"""
GraphSLAM class for simple 2D graph-based SLAM testing.

This version supports adding fixed 2D poses and relative pose constraints,
computing residuals, and preparing for Gauss-Newton optimization.
"""

import numpy as np

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

    def compute_error(self, i, j, z_ij):
        """
        Compute the residual e_ij = z_ij - z_hat_ij
        where z_hat_ij is the predicted relative pose from i to j
        """
        xi = self.poses[i]
        xj = self.poses[j]

        # Convert to transforms
        Ti = v2t(xi)
        Tj = v2t(xj)

        Tij_pred = np.linalg.inv(Ti) @ Tj
        z_hat_ij = t2v(Tij_pred)

        error = z_ij - z_hat_ij
        error[2] = (error[2] + np.pi) % (2 * np.pi) - np.pi
        return error

    def print_summary(self):
        print("Poses:")
        for idx, p in enumerate(self.poses):
            print(f"  x{idx} = {p}")
        print("Constraints:")
        for i, j, z, _ in self.constraints:
            print(f"  x{i} → x{j} : {z}")
