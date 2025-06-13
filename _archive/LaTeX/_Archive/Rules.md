# To Do
1. Add a good testa for subscript problem
2. Fix the LaTeX issues in the example like maing sure matrix rows are in seperate lines
3. Think of a way to combine the rules into a single rule set. after each rule the output can be saved to a code block, and then read in from the code block for the next rule.

### Rule 0: Upload Document

Show the uploaded document in a chat code block

### Rule 1: Inline LaTeX Delimiters

Perform the following tasks on the text in the most recent code block, and place the result in a new code block: 
**Detect Inline LaTeX:**
- Identify LaTeX expressions intended to be inline (on a line with surrounding text).
- If the inline math is not wrapped in **single dollar signs (`$...$`)**, replace the incorrect delimiters (e.g., `\(` and `\)`, or others) with `$`.
- Ensure there are **no spaces inside the delimiters**.
- Ensure there is **at least one space or punctuation mark** separating the math expression from the surrounding text (e.g., `poses $x_i$ and`).

**Example Fix:**

Incorrect: \( x = y \),poses$x=y$and  
Correct: poses $x = y$ and

**Trim Spaces Inside Inline Math Delimiters**

- After converting inline math to use `$...$`, remove **leading and trailing spaces inside** the delimiters.

**Example Fixes:**

Incorrect: $ x + y $  
Correct: $x + y$

**Implementation Note:**  
Before inserting the delimiters, trim whitespace around the expression so that `$ expression $` becomes `$expression$`.


### Rule 1B: External Spacing Around Inline LaTeX

Inline math expressions should be visually and syntactically separated from surrounding text. Without proper spacing, GitHub Markdown may render LaTeX incorrectly or cause unexpected formatting.

#### ✅ Requirements

- Detect inline LaTeX expressions wrapped in `$...$`.
- Ensure there is **at least one space or punctuation mark** before and after the inline expression.
- Add a space **only if** the adjacent character is alphanumeric or underscore.

#### ✅ Example Fixes

```markdown
Incorrect: pose$x_i$and  
Correct: pose $x_i$ and
```

#### 🔍 Implementation Notes

- Do **not** add a space if the surrounding character is already a space or punctuation (`.,;:!?`)  
- Only **one space** is required on each side, even if multiple expressions are adjacent.
- This rule runs **after** Rule 1 to ensure all inline expressions are already delimited correctly.


### Rule 2: Block LaTeX Delimiters

Perform the following tasks on the text in the most recent code block, and place the result in a new code block: 
**Detect Block LaTeX:**

- Identify LaTeX expressions intended as display blocks (on their own lines).
- If the block math is not wrapped in **double dollar signs (`$$...$$`)**, replace the incorrect delimiters (e.g., `\[ ... \]`, or incorrect single `$`) with `$$`.
- Ensure there are **no extra blank lines** within the block.
- Ensure there are **no extra spaces before or after** the `$$` delimiters.

**Example Fix:**

Incorrect:
```
\[
x = y + z
\]
```
Correct:
```
$$
x = y + z
$$
```



**Trim Spaces Inside Block Math Delimiters**

- When wrapping block math expressions with `$$...$$`, ensure there are **no extra spaces or newlines** inside the delimiters.

**Example Fixes:**

Incorrect: $$ x + y $$  
Correct: $$x + y$$

**Implementation Note:**  
Trim both leading/trailing spaces and newlines from the expression before wrapping it in `$$...$$`. This avoids rendering issues in GitHub Markdown.

---

### Rule 7: Avoid Block LaTeX Inside Lists

Block math using `$$...$$` should not be placed directly inside list items (bulleted or numbered), as this breaks rendering in GitHub-flavored Markdown.

**Problem**

GitHub does not reliably render block LaTeX when it appears inside list items like:
```
- This will break:
  $$
  x = y + z
  $$
```

**Approved Workarounds**

1. **Use inline math** if the expression is short:
```
- Update using $x \leftarrow x + \delta$
```
2. **Move the block math outside the list:**
```
- Update the pose estimate.

$$
x \leftarrow x + \delta
$$
```

**Validation Tip**

Check for `$$` blocks that immediately follow list markers like `-`, `*`, or `1.` and ensure they are either converted to inline or moved out of the list structure.

---

### Rule 3: Avoid `_{ij}`

Perform the following task on the text in the most recent code block, and place the result in a new code block: 

Detect LaTeX with the folowing signature used for multiple subscripts _{...}  This can cause rendering issues in GitHub Markdown.  Fix by adding a backslash before the underscore to escape it.

**Example: incorrect**
```
\mathbf{x}_{ij} \quad \mathbf{x}_{ij} \quad \mathbf{x}_{ij}
```

**Example correct**

Add a backslash to fix the problem:
```
\mathbf{x}\_{ij} \quad \mathbf{x}\_{ij} \quad \mathbf{x}\_{ij}
```


# Example LaTeX for Testing
The follow text has embedded latex that will not properly render GitHub. This document will be used to test the LaTeX validation rules.  After appling the rules, the LaTeX should render correctly in GitHub.

### Algorithm Summary

1. **Initialization**

   - Initialize all poses (nodes) with estimates.
   - Construct a factor graph where each edge encodes a relative pose constraint between two poses.

2. **Linearization**

   - For each constraint (edge), compute the error between the predicted and observed relative pose.
   - Linearize the error function using Jacobians \( A_{ij} \) and \( B_{ij} \) with respect to the involved poses \( x_i \) and \( x_j \).

3. **Construct System**

   - Use Jacobians to populate the information matrix \( H \) and vector \( b \):

     \[
     H = J^\top \Omega J \qquad b = J^\top \Omega e
     \]

   - \( \Omega \) is the information (inverse covariance) matrix of the measurement.

4. **Solve**

   - Solve the linear system \( H \delta = -b \) using a sparse solver (e.g., Cholesky decomposition).
   - Update the poses: \( x \leftarrow x + \delta \)

5. **Iterate**

   - Repeat linearization and solving until convergence (change in \( \delta \) is small).

### Pose Definitions

\[
x_i = \begin{bmatrix} x_i \\ y_i \\ \theta_i \end{bmatrix}, \quad
x_j = \begin{bmatrix} x_j \\ y_j \\ \theta_j \end{bmatrix}, \quad
\Delta t = \begin{bmatrix} x_j - x_i \\ y_j - y_i \end{bmatrix}
\]

Let the rotation matrix of pose \( x_i \) be:

\[
R_i \in \mathbb{R}^{2 \times 2}
\]

and define the skew-symmetric matrix:

\[
S = \begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
\]

### Error Function and Observations

Each edge in the graph represents a constraint based on a relative pose observation:

\[
\mathbf{z}_{ij} = 
\begin{bmatrix} 
\Delta\mathbf{x}_{ij} \ \Delta\mathbf{y}_{ij} \\ 
\Delta y_{ij} \ \Delta\theta_{ij} \end{bmatrix}
\]

This observation represents the expected transformation from node \( i \) to node \( j \), measured in the coordinate frame of node \( i \).

The predicted relative pose based on current estimates is:

\[
\hat{z}_{ij} = \begin{bmatrix} 
R_i^\top (t_j - t_i) \\ 
\theta_j - \theta_i \end{bmatrix}
\]

The error function is the difference between the observed and predicted relative pose:

\[
\mathbf{e}_{ij} = \mathbf{z}_{ij} - \hat{z}_{ij}
\]

This error is used in the least-squares cost function that drives the optimization.

### Jacobians

The Jacobians of this function are with respect to the two involved poses \( x_i \) and \( x_j \).

\[
A_{ij} =
\begin{bmatrix}
-R_i^\top & R_i^\top S \Delta t \\
0_{1 \times 2} & -1
\end{bmatrix}
\]

\[
B_{ij} =
\begin{bmatrix}
R_i^\top & 0_{2 \times 1} \\
0_{1 \times 2} & 1
\end{bmatrix}
\]

### Summary

- These Jacobians are used to populate the sparse system matrix \( H \) and vector \( b \) in SLAM optimization.
- The rotation matrix \( R_i^\top \) transforms global coordinates to local frame \( i \).
- The skew-symmetric matrix \( S \) arises from the derivative of a rotation operation.

---
<br><br><br><br>

