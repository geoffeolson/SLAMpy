## Constructing the Linear System

This section derives the linear system used in Gauss-Newton optimization for Graph-Based SLAM. The goal is to construct the approximate Hessian matrix \( H \) and the gradient vector \( b \) from all pairwise constraints.

---

### Cost Function

We minimize the total weighted squared error:

$$
\min_{\delta \mathbf{x}} \sum_{(i,j) \in \mathcal{C}} \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)^T \Omega_{ij} \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)
$$

At each iteration, we linearize this objective and solve:

$$
H \, \delta \mathbf{x} = -b
$$

Where:
- \( H \) is the system matrix (approximate Hessian).
- \( b \) is the system vector (negative gradient).
- \( \delta \mathbf{x} \) is the pose update vector.

---

### Contribution from a Single Constraint

For a constraint \( (i,j) \), let the Jacobians be:

$$
J_{ij} = \begin{bmatrix} A_{ij} & B_{ij} \end{bmatrix}
$$

Then:

$$
H_{ij} = J_{ij}^T \, \Omega_{ij} \, J_{ij}
$$

$$
b_{ij} = J_{ij}^T \, \Omega_{ij} \, \mathbf{e}_{ij}
$$

These represent local contributions to the global system. The blocks are added to the matrix \( H \) and vector \( b \) at the appropriate positions corresponding to poses \( \mathbf{x}_i \) and \( \mathbf{x}_j \).

---

### Expanded Block Form

If the poses are 3D (e.g., SE(2)), each Jacobian is a 3×3 matrix. Then:

$$
H_{ij} =
\begin{bmatrix}
A_{ij}^T \Omega A_{ij} & A_{ij}^T \Omega B_{ij} \\
B_{ij}^T \Omega A_{ij} & B_{ij}^T \Omega B_{ij}
\end{bmatrix}, \quad
b_{ij} =
\begin{bmatrix}
A_{ij}^T \Omega \, \mathbf{e}_{ij} \\
B_{ij}^T \Omega \, \mathbf{e}_{ij}
\end{bmatrix}
$$

---

### Notes

- \( \Omega_{ij} \) is the information matrix (inverse of the measurement covariance).
- Each constraint only affects the sub-blocks of \( H \) and \( b \) related to poses \( i \) and \( j \).
- The sparsity of \( H \) is determined by the connectivity of the pose graph.

This system is solved iteratively using sparse linear solvers (e.g., Cholesky decomposition) to refine the pose estimates.