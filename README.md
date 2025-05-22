# Graph-Based SLAM: Gauss-Newton Optimization Algorithm (Simplified and Complete)

## Overview
This document describes the iterative optimization algorithm used for Graph-Based SLAM, based on the Grisetti et al. (2010) tutorial. The algorithm estimates robot poses that minimize the total squared error from relative motion and measurement constraints. The original description (Algorithm 1) lacked some practical implementation details, which are included here.

---

## Function: `compute_jacobians()`

### 1. Purpose
Computes the Jacobians of the residual error function with respect to two poses connected by a constraint. Used in constructing the system matrix `H` and vector `b` for linear optimization.

### 2. Inputs and Outputs
- **Input**: A constraint tuple `(i, j, z_ij, Ω_ij)`
- **Output**: Matrices `A_ij = ∂e/∂x_i` and `B_ij = ∂e/∂x_j`, each 3×3

### 3. Equation

<img src="https://latex.codecogs.com/svg.latex?\mathbf{e}_{ij}%20=%20\mathrm{t2v}(T_i^{-1}%20T_j)%20-%20\mathbf{z}_{ij}" title="e_ij = t2v(Ti^{-1} Tj) - z_ij" />

### 4. Breakdown

Let:

<img src="https://latex.codecogs.com/svg.latex?x_i=egin{bmatrix}x_i\y_i\	heta_i\end{bmatrix},\quad
x_j=egin{bmatrix}x_j\y_j\	heta_j\end{bmatrix},\quad
\Delta t = egin{bmatrix}x_j - x_i\y_j - y_i\end{bmatrix}" title="Pose vector definitions and delta t" />

Then compute Jacobians:

**A_ij:**

<img src="https://latex.codecogs.com/svg.latex?A_{ij}[0:2,0:2]=-R_i^	op,\quad
A_{ij}[0:2,2]=R_i^	op
egin{bmatrix}0&-1\1&0\end{bmatrix}
\Delta t,\quad
A_{ij}[2,2]=-1" title="A_ij Jacobian equations" />

**B_ij:**

<img src="https://latex.codecogs.com/svg.latex?B_{ij}[0:2,0:2]=R_i^	op,\quad
B_{ij}[2,2]=1" title="B_ij Jacobian equations" />

---

## Numerical Jacobian Testing

To verify the correctness of analytical Jacobians, numerical derivatives are computed as follows:

<img src="https://latex.codecogs.com/svg.latex?rac{\partial{e}}{\partial{x_k}}%20pprox%20rac{e(x+\epsilon)-e(x)}{\epsilon}" title="Numerical derivative formula" />

---

## Notes
Each constraint is structured as:
```python
(i, j, z_ij, Omega_ij)
```
Used to compute residuals and derivatives required for optimization.
