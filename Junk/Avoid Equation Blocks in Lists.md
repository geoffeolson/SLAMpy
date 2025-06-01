## Markdown Equation Block Validator – Rule 7 (v1.1)

This document defines **Rule 7**, focused on LaTeX formatting inside Markdown lists.

---

### ❌ Rule 7: Avoid `$$...$$` Block Equations Inside Lists

**Issue:**
Block equations inside bullet or numbered lists often fail to render properly in GitHub Markdown. They may break list formatting or render unpredictably.

---

### 🔍 Validation Procedure

1. **Search for Problem:**

   * Identify any instances of `$$...$$` blocks that appear **immediately under list items** (e.g., `-`, `*`, `1.`) without breaking the list structure.

2. **Fix the Problem:**

   * Convert the equation into **inline math** using single dollar signs:

     ```markdown
     - Update using $x \leftarrow x + \delta$
     ```
   * If inline math is not sufficient and you require a block equation, consider placing the equation outside the list structure to preserve hierarchy

     ```markdown
     - Update the pose estimate.

     $$
     x \leftarrow x + \delta
     $$
     ```

---

✅ This validator detects and corrects structural LaTeX issues that may cause GitHub to misrender list formatting.
