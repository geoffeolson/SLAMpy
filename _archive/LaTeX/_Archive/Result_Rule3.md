## A Simple Graph-Based SLAM

This document explains the derivation of the Jacobians used in a simple Graph-Based SLAM implementation. It includes pose definitions, matrix notation, and the role of Jacobians in constructing the linear system for optimization.

---

### Algorithm Summary

1. **Initialization**

   - Initialize all poses (nodes) with estimates.
   - Construct a factor graph where each edge encodes a relative pose constraint between two poses.

2. **Linearization**

   - For each constraint (edge), compute the error between the predicted and observed relative pose.
   - Linearize the error function using Jacobians $A\_{ij}$ and $B\_{ij}$ with respect to the involved poses $x\_i$ and $x\_j$.

3. **Construct System**

   - Use Jacobians to populate the information matrix $H$ and vector $b$:

   $$
   H = J^\top \Omega J \qquad b = J^\top \Omega e
   $$

   - $\Omega$ is the information (inverse covariance) matrix of the measurement.

4. **Solve**

   - Solve the linear system $H \delta = -b$ using a sparse solver (e.g., Cholesky decomposition).
   - Update the poses: $x \leftarrow x + \delta$

5. **Iterate**

   - Repeat linearization and solving until convergence (change in $\delta$ is small).

---

### Pose Definitions

$$
x\_i = \begin{bmatrix} x\_i \\ y\_i \\ \theta\_i \end{bmatrix}, \quad
x\_j = \begin{bmatrix} x\_j \\ y\_j \\ \theta\_j \end{bmatrix}, \quad
\Delta t = \begin{bmatrix} x\_j - x\_i \\ y\_j - y\_i \end{bmatrix}
$$

Let the rotation matrix of pose $x\_i$ be:

$$
R\_i \in \mathbb{R}^{2 \times 2}
$$

and define the skew-symmetric matrix:

$$
S = \begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
$$

---

### Error Function and Observations

Each edge in the graph represents a constraint based on a relative pose observation:

$$
z\_{ij} = \begin{bmatrix} \Delta x\_{ij} \\ \Delta y\_{ij} \\ \Delta \theta\_{ij} \end{bmatrix}
$$

This observation represents the expected transformation from node $i$ to node $j$, measured in the coordinate frame of node $i$.

The predicted relative pose based on current estimates is:

$$
\hat{z}\_{ij} = \begin{bmatrix} R\_i^\top (t\_j - t\_i) \\ \theta\_j - \theta\_i \end{bmatrix}
$$

The error function is the difference between the observed and predicted relative pose:

$$
e\_{ij} = z\_{ij} - \hat{z}\_{ij}
$$

This error is used in the least-squares cost function that drives the optimization.

---

### Jacobians

The Jacobians of this function are with respect to the two involved
