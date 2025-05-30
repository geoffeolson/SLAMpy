## A Simple Graph-Based SLAM

This document explains the derivation of the Jacobians used in a simple Graph-Based SLAM implementation. It includes pose definitions, matrix notation, and the role of Jacobians in constructing the linear system for optimization.

---

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
     H = J^	op \\Omega J \\qquad b = J^	op \\Omega e
     $$

   - $\\Omega$ is the information (inverse covariance) matrix of the measurement.

4. **Solve**

   - Solve the linear system $H \\delta = -b$ using a sparse solver (e.g., Cholesky decomposition).
   - Update the poses: $x \\leftarrow x + \\delta$

5. **Iterate**

   - Repeat linearization and solving until convergence (change in $\\delta$ is small).

---

### Pose Definitions

$$
x_i = egin{bmatrix} x_i \\\\ y_i \\\\ 	heta_i \\end{bmatrix}, \\quad
x_j = egin{bmatrix} x_j \\\\ y_j \\\\ 	heta_j \\end{bmatrix}, \\quad
\\Delta t = egin{bmatrix} x_j - x_i \\\\ y_j - y_i \\end{bmatrix}
$$

Let the rotation matrix of pose $x_i$ be:

$$
R_i \\in \\mathbb{R}^{2 	imes 2}
$$

and define the skew-symmetric matrix:

$$
S = egin{bmatrix}
0 & -1 \\\\
1 & 0
\\end{bmatrix}
$$

---

### Error Function and Observations

Each edge in the graph represents a constraint based on a relative pose observation:

$$
z_{ij} = egin{bmatrix} \\Delta x_{ij} \\\\ \\Delta y_{ij} \\\\ \\Delta 	heta_{ij} \\end{bmatrix}
$$

This observation represents the expected transformation from node $i$ to node $j$, measured in the coordinate frame of node $i$.

The predicted relative pose based on current estimates is:

$$
\\hat{z}_{ij} = egin{bmatrix} R_i^	op (t_j - t_i) \\\\ 	heta_j - 	heta_i \\end{bmatrix}
$$

The error function is the difference between the observed and predicted relative pose:

$$
e_{ij} = z_{ij} - \\hat{z}_{ij}
$$

This error is used in the least-squares cost function that drives the optimization.

---

### Jacobians

The Jacobians of this function are with respect to the two involved poses $x_i$ and $x_j$.

$$
A_{ij} =
egin{bmatrix}
-R_i^	op & R_i^	op S \\Delta t \\\\
0_{1 	imes 2} & -1
\\end{bmatrix}
$$

$$
B_{ij} =
egin{bmatrix}
R_i^	op & 0_{2 	imes 1} \\\\
0_{1 	imes 2} & 1
\\end{bmatrix}
$$

---

### Summary

- These Jacobians are used to populate the sparse system matrix $H$ and vector $b$ in SLAM optimization.
- The rotation matrix $R_i^	op$ transforms global coordinates to local frame $i$.
- The skew-symmetric matrix $S$ arises from the derivative of a rotation operation.