## Markdown Equation Block Validator – Rule 10 (v1.0)

This document defines **Rule 10**, which checks for corrupted LaTeX escape sequences in Markdown files.

---

### ❌ Rule 10: Avoid Backslash Escape Sequences in Exported Files

**Issue:**
Certain escape sequences like `\t`, `\n`, `\b`, `\f`, or `\a` may be misinterpreted during file export or by some text editors, especially Notepad++. This can corrupt LaTeX commands in Markdown documents.

---

### 🔍 Validation Procedure

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

✅ This validator detects common LaTeX control character corruptions and helps restore correct syntax before GitHub upload.
