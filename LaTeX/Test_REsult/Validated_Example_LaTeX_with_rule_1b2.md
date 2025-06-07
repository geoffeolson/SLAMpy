# Example LaTeX for Testing
The follow text has embedded latex that will not properly render GitHub. This document will be used to test the LaTeX validation rules.  After appling the rules, the LaTeX should render correctly in GitHub.

### Algorithm Summary

1. **Initialization**

   - Initialize all poses (nodes) with estimates.
   - Construct a factor graph where each edge encodes a relative pose constraint between two poses.

2. **Linearization**

   - For each constraint (edge), compute the error between the predicted and observed relative pose.
   - Linearize the error function using Jacobians $A_{ij}$ and $B_{ij}$ with respect to the involved poses $x_i$ and $x_j$.

3. **Construct System**

   - Use Jacobians to populate the information matrix $H$ and vector $b$:

$$
H = J^\top \Omega J \qquad b = J^\top \Omega e
$$

   - $\Omega$ is the information (inverse covariance) matrix of the measurement.

4. **Solve**

   - Solve the linear system $H \delta = -b$ using sparse Cholesky decomposition.
   - Update poses: $x_i \leftarrow x_i \oplus \delta_i$

### Pose Definitions

We define the poses and their relative translation as:

$$
\mathbf{x}_{i} = \begin{bmatrix} x_i \\ y_i \\ \theta_i \end{bmatrix}, \quad
\mathbf{x}_{j} = \begin{bmatrix} x_j \\ y_j \\ \theta_j \end{bmatrix}, \quad
\Delta \mathbf{t} = \begin{bmatrix} x_j - x_i \\ y_j - y_i \end{bmatrix}
$$

Let $R_i \in \mathbb{R}^{2 \times 2}$ be the rotation matrix of pose $\mathbf{x}_{i}$, and define the skew-symmetric matrix:

$$
S = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

### Error Function and Observations

Each edge in the graph represents a constraint based on a relative pose observation:

$$
\mathbf{z}_{ij} =
\begin{bmatrix}
\Delta\mathbf{x}_{ij} \ \Delta\mathbf{y}_{ij} \\
\Delta y_{ij} \ \Delta\theta_{ij}
\end{bmatrix}
$$

The error function is defined as:

$$
\mathbf{e}_{ij}(\mathbf{x}_{i}, \mathbf{x}_{j}) =
R_i^\top (\mathbf{t}_j - \mathbf{t}_i) - \mathbf{t}_{ij}
$$

Repeated errors: $\mathbf{e}_{ij}$ $\mathbf{e}_{ij}$ $\mathbf{e}_{ij}$

Broken form (should be converted by Rule 3): $\mathbf{x}\_{ij}$ $\mathbf{x}\_{ij}$

Fixed form: $\mathbf{x_{ij}}$ $\mathbf{x_{ij}}$
