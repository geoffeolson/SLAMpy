## Markdown Equation Block Validator – Rule 4 (v1.0)

This document defines **Rule 4** of the Markdown LaTeX validator system. Rule 4 addresses an issue where **commas separating multiple matrix expressions** in a single math block can prevent correct rendering in GitHub-flavored Markdown.

---

### ❌ Rule 4: Avoid Using Commas Between Matrix Expressions in a Block

**Issue:**
LaTeX blocks with multiple matrix expressions separated by commas often fail to render in GitHub:
```latex
\begin{bmatrix} a \\ b \end{bmatrix}, \begin{bmatrix} c \\ d \end{bmatrix}
```
This will frequently break rendering.

---

### ✅ Approved Workarounds:
Use one of the following instead:
1. Place matrices on **separate lines**:
```latex
\begin{bmatrix} a \\ b \end{bmatrix} \\
\begin{bmatrix} c \\ d \end{bmatrix}
```
2. Place each matrix in its **own equation block**:
```markdown
$$
\begin{bmatrix} a \\ b \end{bmatrix}
$$

$$
\begin{bmatrix} c \\ d \end{bmatrix}
$$
```

---

### 🔍 Detection Pattern (Regex):
Detects commas between consecutive `\begin{...}` blocks:
```
\end\{[a-zA-Z]+\}\s*,\s*\begin\{[a-zA-Z]+\}
```

---

### ✅ Summary:
- Do not separate matrix or multiline expressions with commas in a single LaTeX block
- Use separate lines (`\\`) or break into multiple equation blocks (`$$ ... $$`)

---

**Status:** This rule prevents a GitHub rendering failure caused by improper formatting of multiline expressions using commas.
