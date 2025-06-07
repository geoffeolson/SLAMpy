### Example LaTeX for Testing

This section includes LaTeX formatting issues commonly produced by ChatGPT. It is used for validating formatting rules.

---

### Pose Definitions

We define the poses and their relative translation as:

$$
\mathbf{x}_{i} = \begin{bmatrix} x_i \\ y_i \\ \theta_i \end{bmatrix}, \quad
\mathbf{x}_{j} = \begin{bmatrix} x_j \\ y_j \\ \theta_j \end{bmatrix}, \quad
\Delta \mathbf{t} = \begin{bmatrix} x_j - x_i \\ y_j - y_i \end{bmatrix}
$$

Let \( R_i \in \mathbb{R}^{2 \times 2} \) be the rotation matrix of pose \( \mathbf{x}_i \), and define the skew-symmetric matrix:

$$
S = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

---

### Jacobians

The Jacobians with respect to the poses \( \mathbf{x}_i \) and \( \mathbf{x}_j \) are:

$$
A_{ij}[0:2,0:2] = -R_i^\top, \quad
A_{ij}[0:2,2] = R_i^\top
\begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
\Delta t
$$

$$
B_{ij}[0:2,0:2] = R_i^\top, \quad
B_{ij}[2,2] = 1
$$

---

### Error Function and Observations

Each edge in the graph represents a constraint based on a relative pose observation:

$$
\mathbf{z}_{ij} = 
\begin{bmatrix} 
\Delta\mathbf{x}_{ij} \ \Delta\mathbf{y}_{ij} \\
\Delta y_{ij} \ \Delta\theta_{ij}
\end{bmatrix}
$$

---

### Inline Formatting

This is an inline expression: \( x_i \), and this is another one: \( \mathbf{x}_j \).

Check spacing around expressions: poses \( x_i \) and \( x_j \) are adjacent.

This line should fix spacing: poses \(x_i\) and \(x_j\) are adjacent.

---

### Repeated Subscripts (GitHub Bug Workaround)

This fails on GitHub:

$$
\mathbf{x}_{ij} \quad \mathbf{x}_{ij} \quad \mathbf{x}_{ij}
$$

Approved form that renders correctly:

$$
\mathbf{x_{ij}} \quad \mathbf{x_{ij}} \quad \mathbf{x_{ij}}
$$
