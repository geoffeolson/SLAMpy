### Observation Constraints

Observation constraints are added when the robot observes a known landmark. These constraints relate a single robot pose to a landmark position using a measurement model.

Each observation provides a constraint between a robot pose $\mathbf{x}\_i$ and a known landmark position $\mathbf{l}\_k$. The error function compares the expected observation (based on the pose and the landmark) with the actual measurement.

#### Definitions

Let this be the robot pose at time $i$:

$$
\mathbf{x}\_i = \begin{bmatrix} 
x\_i \\ 
y\_i \\ 
\theta\_i 
\end{bmatrix}
$$

let this be the position of landmark $k$

$$
\mathbf{l}\_k = \begin{bmatrix} 
x\_k \\ 
y\_k 
\end{bmatrix}
$$ 

Let this be the observed relative position from pose $i$ to landmark $k$, in the robot's reference frame:

$$
\mathbf{z}\_{ik} = 
\begin{bmatrix} 
\Delta x \\ 
\Delta y 
\end{bmatrix}
$$ 

The expected observation $\hat{\mathbf{z}}\_{ik}$ is computed by:

$$
\hat{\mathbf{z}}\_{ik} = R(\theta\_i)^\top \cdot \left( \mathbf{l}\_k - \mathbf{t}\_i \right)
$$

where: 

$$
\mathbf{t}\_i = \begin{bmatrix} x\_i \\ 
y\_i \end{bmatrix}
$$ 

is the translation part of pose $\mathbf{x}\_i$ and $R(\theta\_i)$ is the 2D rotation matrix:

$$
R(\theta\_i) = 
\begin{bmatrix}
\cos\theta\_i & -\sin\theta\_i \\
\sin\theta\_i & \cos\theta\_i
\end{bmatrix}
$$

#### Error Function

The error is defined as the difference between the predicted and observed measurements:

$$
\mathbf{e}\_{ik} = \hat{\mathbf{z}}\_{ik} - \mathbf{z}\_{ik}
$$

#### Jacobian

The Jacobian $\mathbf{J}\_{ik}$ is the derivative of the error function with respect to the pose $\mathbf{x}\_i$.

Let:

- $\Delta x = x\_k - x\_i$
- $\Delta y = y\_k - y\_i$
- $c = \cos\theta\_i$
- $s = \sin\theta\_i$

Then the Jacobian is:

$$
\mathbf{J}\_{ik} = \frac{\partial \mathbf{e}\_{ik}}{\partial \mathbf{x}\_i} = \begin{bmatrix}
\-c & - s & - s \cdot \Delta x + c \cdot \Delta y \\
s & - c & - c \cdot \Delta x - s \cdot \Delta y
\end{bmatrix}
$$

Note that this is a $2 \times 3$ matrix since the observation error is 2D, and the pose has 3 parameters.

#### Linear System Contribution

Each observation constraint contributes to the linear system:

- Error vector $\mathbf{e}\_{ik}$ is added to the stacked residual vector $\mathbf{b}$
- Jacobian $\mathbf{J}\_{ik}$ is used to form the local information matrix:

$$
\mathbf{H}\_{ii} += \mathbf{J}\_{ik}^\top \, \Omega\_{ik} \, \mathbf{J}\_{ik}
$$

- And the right-hand side vector:

$$
\mathbf{b}\_i += \mathbf{J}\_{ik}^\top \, \Omega\_{ik} \, \mathbf{e}\_{ik}
$$

Here $\Omega\_{ik} = \Sigma\_{ik}^{-1}$ is the information matrix associated with the measurement noise.

#### Summary

- Observation constraints connect one robot pose $\mathbf{x}\_i$ to one known landmark $\mathbf{l}\_k$
- The error is defined in rectangular (x, y) coordinates
- Only one block in $\mathbf{H}$ (the pose block) is updated, since the landmark positions are assumed fixed
- This formulation avoids dependencies between multiple robot poses, unlike motion constraints
