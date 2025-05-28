# A Simple Graph-Based SLAM

This document explains the derivation of the Jacobians used in a simple Graph-Based SLAM implementation. It includes pose definitions, matrix notation, and the role of Jacobians in constructing the linear system for optimization.

---

### Algorithm Summary

1. **Initialization**
   - Initialize all poses (nodes) with estimates.
   - Construct a factor graph where each edge encodes a relative pose constraint between two poses.

2. **Linearization**
   - For each constraint (edge), compute the error between the predicted and observed relative pose.
   - Linearize the error function using Jacobians $A_{ij}$ and $B_{ij}$ with respect to the involved poses $\mathbf{x}_i$ and $\mathbf{x}_j$.

3. **Construct System**
   - Use Jacobians to populate the information matrix $H$ and vector $b$: $\quad H = J^\top \Omega J \qquad b = J^\top \Omega e$
   - $\Omega$ is the information (inverse covariance) matrix of the measurement.

4. **Solve**
   - Solve the linear system $H \delta = -b$ using a sparse solver (e.g., Cholesky decomposition).
   - Update the poses: $\mathbf{x} \leftarrow \mathbf{x} + \delta$

5. **Iterate**
   - Repeat linearization and solving until convergence (change in $\delta$ is small).

---

### Pose Definitions

We define the poses and their relative translation as:

$$
\mathbf{x}_i = \begin{bmatrix} x_i \\ y_i \\ \theta_i \end{bmatrix}, \quad
\mathbf{x}_j = \begin{bmatrix} x_j \\ y_j \\ \theta_j \end{bmatrix}, \quad
\Delta \mathbf{t} = \begin{bmatrix} x_j - x_i \\ y_j - y_i \end{bmatrix}
$$

Let $R_i \in \mathbb{R}^{2 	imes 2}$ be the rotation matrix of pose $\mathbf{x}_i$, and define the skew-symmetric matrix:

$$
S = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

---

### Error Function and Observations

Each edge in the graph represents a constraint based on a relative pose observation:

$$
\mathbf{z}_{ij}=\begin{bmatrix}
\Delta &xij\\
\Delta &xij\\
\Delta &\theta{ij}
\end{bmatrix}
$$

This observation represents the expected transformation from node $i$ to node $j$, measured in the coordinate frame of node $i$.

The predicted relative pose based on current estimates is:

$$
\hat{\mathbf{z}}_{ij}=\begin{bmatrix}R_i^op(\mathbf{t}_j-\mathbf{t}_i)\\theta_j-theta_i\end{bmatrix}
$$

The error function is the difference between the observed and predicted relative pose:

$$
e_{ij} = z_{ij} - \hat{\mathbf{z}}_{ij}
$$

---

### Jacobians

The error function expresses the discrepancy between the observed relative pose and the predicted relative pose. The Jacobians of this function are with respect to the two involved poses: $\mathbf{x}_i$ and $\mathbf{x}_j$.

$$
A_{ij} =
\begin{bmatrix}
R_i^\top & R_i^\top S \Delta \mathbf{t} \\
\mathbf{0}^\top & -1
\end{bmatrix}
$$

$$
B_{ij} =
\begin{bmatrix}
R_i^\top & \mathbf{0}^\top \\
\mathbf{0}^\top & 1
\end{bmatrix}
$$

- The top-left blocks represent the partial derivatives with respect to position.
- The right column in $A_{ij}$ and the bottom row of both matrices capture orientation influence.

---

### Summary

- These Jacobians are used to populate the sparse system matrix $H$ and vector $b$ in SLAM optimization.
- They are essential for linearizing the error function around current estimates.
- The rotation matrix $R_i^	op$ transforms global coordinates to local frame $i$.
- The skew-symmetric matrix $S$ arises from the derivative of a rotation operation.

This derivation and breakdown align with standard formulations used in SE(2)-based SLAM literature.

---

### Additional Resources

- **Paper Reference**: Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010). *A Tutorial on Graph-Based SLAM*. IEEE ITS Magazine.
- **Tools**: Python, NumPy, Matplotlib, GTSAM.

---

### How to Run

1. Install dependencies:
   ```bash
   pip install numpy matplotlib
   ```
2. Run the script:
   ```bash
   python slam_11_a_simple_graph_slam.py
   ```
