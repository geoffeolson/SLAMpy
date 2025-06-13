# Latex Conversion Workflow for Markdown Files
This describes a workflow for converting the LaTeX in a Markdown file (.md) from the format used by ChatGPT to a format that will render in GitHub.  Open the Markdown file, perform the tasks below on the document, and close the document.

### Task 1: Inline LaTeX Delimiters

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


### Task 2: Block LaTeX Delimiters

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

Incorrect: - $ \Omega $ is  
Correct: - $\Omega$ is

**Implementation Note:**  
Trim both leading/trailing spaces and newlines from the expression before wrapping it in `$$...$$`. This avoids rendering issues in GitHub Markdown.

### Task 3: Subscripts

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



