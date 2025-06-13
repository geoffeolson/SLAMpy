
This section derives the linear system used in Gauss-Newton optimization for Graph-Based SLAM.

We compute the residual vector:
$$
\mathbf{e} \_{ij} = \mathbf{x}_j - \mathbf{x}_i, \quad \mathbf{e} \_{ij}
$$

From this, we construct the vector b:
$$
b_{ij} = A_{ij}^T \Omega \mathbf{e} \_{ij} + B_{ij}^T \Omega \mathbf{e} \_{ij}
$$
