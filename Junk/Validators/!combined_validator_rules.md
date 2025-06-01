## GitHub Markdown Equation Validators (v1.0)

This document defines a set of validation rules to detect and correct LaTeX formatting issues in Markdown files intended for GitHub rendering.

---

### ✅ Rule 1: Use Double Dollar Signs for Block Equations

**Issue:**
GitHub Markdown does not render block equations that are surrounded by single dollar signs (`$...$`). Only double dollar signs (`$$...$$`) should be used for block-level LaTeX.

#### 🔍 Validation Procedure

1. **Search for Problem:**
   - Find any block math delimited by **single** dollar signs on a separate line.
     ```markdown
     $
     x = a + b
     $
     ```

2. **Fix the Problem:**
   - Replace single dollar sign delimiters with double dollar signs:
     ```markdown
     $$
     x = a + b
     $$
     ```

---

### ✅ Rule 7: Avoid Block Equations Inside List Items

**Issue:**
Block equations (`$$...$$`) placed inside bullet or numbered list items often break GitHub's Markdown rendering.

#### 🔍 Validation Procedure

1. **Search for Problem:**
   - Detect list items followed immediately by a block equation.
     ```
     - List item text
       $$
       math block
       $$
     ```

2. **Fix the Problem:**
   - Move the block equation **outside** the list structure.
     ```
     - List item text

       $$
       math block
       $$
     ```

---

### ✅ Rule 10: Avoid Backslash Escape Sequences in Exported Files

**Issue:**
Certain escape sequences like `\t`, `\n`, `\b`, `\f`, or `\a` may be misinterpreted during file export or by some text editors, especially Notepad++. This can corrupt LaTeX commands in Markdown documents.

#### 🔍 Validation Procedure

1. **Search for Problem:**
   - Look for corrupted LaTeX commands like:
     - `\b egin{bmatrix}` → should be `\begin{bmatrix}`
     - `\f rac{}` → should be `\frac{}`
     - `\t imes` → should be `\times`
     - `\a lpha` → should be `\alpha`
     - `\a rray` → should be `\array`

2. **Fix the Problem:**
   - Use search-and-replace to correct the spacing issues:
     - Replace `\b egin` → `\begin`
     - Replace `\f rac` → `\frac`
     - Replace `\t imes` → `\times`
     - Replace `\a lpha` → `\alpha`
     - Replace `\a rray` → `\array`

---

### ✅ Rule 3: Avoid Multiple Subscripts with `\mathbf{}` in a Single Block

**Issue:**
GitHub Markdown may fail to render equations correctly if `\mathbf{}` is used with multiple indexed variables in the same LaTeX block. This may work once, but repeated uses in the same block often cause rendering failures.

#### 🔍 Validation Procedure

1. **Search for Problem:**
   - Look for expressions like `\mathbf{e}_{ij}` repeated more than once in the same `$$...$$` block.

2. **Fix the Problem:**
   - Replace them with a workaround that splits the subscript using a space and backslash:
     - Replace `\mathbf{e}_{ij}` with `\mathbf{e} \_{ij}`

---

### ✅ Rule 4: Avoid Comma-Separated Equations in LaTeX Blocks

**Issue:**
In GitHub-flavored Markdown, using commas to separate multiple equations within a `$$...$$` block may prevent correct rendering.

#### 🔍 Validation Procedure

1. **Search for Problem:**
   - Locate lines inside `$$...$$` blocks that contain commas separating equations, like:
     ```latex
     $$
     x = a + b,\quad y = c + d
     $$

2. **Fix the Problem:**
   - Break these into separate LaTeX blocks:
     ```latex
     $$
     x = a + b
     $$
     $$
     y = c + d
     $$
     ```

✅ These validators are designed to be used together to ensure GitHub-compatible LaTeX in Markdown files. Additional rules can be appended using the same format.
