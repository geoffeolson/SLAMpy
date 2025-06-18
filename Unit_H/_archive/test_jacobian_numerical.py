import numpy as np
from numpy import sin, cos, pi, eye
from copy import deepcopy

def v2t(pose_vec):
    x, y, theta = pose_vec
    return np.array([
        [cos(theta), -sin(theta), x],
        [sin(theta),  cos(theta), y],
        [0,          0,           1]
    ])

def t2v(T):
    x = T[0, 2]
    y = T[1, 2]
    theta = np.arctan2(T[1, 0], T[0, 0])
    return np.array([x, y, theta])

class GraphSLAM:
    def __init__(self):
        self.poses = []
        self.constraints = []

    def add_pose(self, pose):
        self.poses.append(np.array(pose))

    def add_constraint(self, i, j, z_ij, Omega_ij):
        self.constraints.append((i, j, np.array(z_ij), np.array(Omega_ij)))

    def compute_error(self, constraint):
        i, j, z_ij, _ = constraint
        Ti = v2t(self.poses[i])
        Tj = v2t(self.poses[j])
        Tij_pred = np.linalg.inv(Ti) @ Tj
        z_hat_ij = t2v(Tij_pred)
        e = z_ij - z_hat_ij
        e[2] = (e[2] + pi) % (2 * pi) - pi
        return e

    def compute_jacobians(self, constraint):
        i, j, z_ij, _ = constraint
        xi = self.poses[i]
        xj = self.poses[j]

        Ti = v2t(xi)
        Tj = v2t(xj)
        Ri = Ti[0:2, 0:2]
        ti = xi[0:2]
        tj = xj[0:2]
        delta_t = tj - ti

        A = np.zeros((3, 3))
        B = np.zeros((3, 3))

        A[0:2, 0:2] = -Ri.T
        A[0:2, 2] = Ri.T @ np.array([[0, -1], [1, 0]]) @ delta_t
        A[2, 2] = -1.0

        B[0:2, 0:2] = Ri.T
        B[2, 2] = 1.0

        return A, B

def numerical_jacobian(gs, constraint, epsilon=1e-6):
    i, j, _, _ = constraint
    e0 = gs.compute_error(constraint)
    A_num = np.zeros((3, 3))
    B_num = np.zeros((3, 3))

    for k in range(3):
        perturbed = deepcopy(gs)
        perturbed.poses[i][k] += epsilon
        ei = perturbed.compute_error(constraint)
        A_num[:, k] = (ei - e0) / epsilon

    for k in range(3):
        perturbed = deepcopy(gs)
        perturbed.poses[j][k] += epsilon
        ej = perturbed.compute_error(constraint)
        B_num[:, k] = (ej - e0) / epsilon

    return A_num, B_num

if __name__ == '__main__':
    # Initialize test
    gs = GraphSLAM()
    gs.add_pose([0.0, 0.0, 0.0])
    gs.add_pose([1.0, 0.1, 5 * pi / 180])
    gs.add_constraint(0, 1, [1.0, 0.0, 0.0], eye(3))

    constraint = gs.constraints[0]
    A_ana, B_ana = gs.compute_jacobians(constraint)
    A_num, B_num = numerical_jacobian(gs, constraint)

    print("Analytical A:")
    print(A_ana)
    print("Numerical A:")
    print(A_num)
    print("Difference A:")
    print(A_ana - A_num)

    print("\nAnalytical B:")
    print(B_ana)
    print("Numerical B:")
    print(B_num)
    print("Difference B:")
    print(B_ana - B_num)
