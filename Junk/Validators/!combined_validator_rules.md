## Latex Rule1 Validator (Updated)

This validator rule ensures that LaTeX equations in a Markdown file use the correct delimiters for GitHub rendering **without introducing unwanted spaces**, and that inline math expressions are **surrounded by appropriate spacing** to separate them from regular text.

---

### ✅ Rule 1: Correct LaTeX Delimiters and Spacing

#### 🔍 Validation Logic

1. **Detect Inline LaTeX:**
   - Identify LaTeX expressions intended to be inline (on a line with surrounding text).
   - If the inline math is not wrapped in **single dollar signs (`$...$`)**, replace the incorrect delimiters (e.g., `\(` and `\)`, or others) with `$`.
   - Ensure there are **no spaces inside the delimiters**.
   - Ensure there is **at least one space or punctuation mark** separating the math expression from the surrounding text (e.g., `poses $x_i$ and`).

   **Example Fix:**
   ```markdown
   Incorrect: \( x = y \),poses$x=y$and
   Correct: poses $x = y$ and
   ```

2. **Detect Block LaTeX:**
   - Identify LaTeX expressions intended as display blocks (on their own lines).
   - If the block math is not wrapped in **double dollar signs (`$$...$$`)**, replace the incorrect delimiters (e.g., `\[ ... \]`, or incorrect single `$`) with `$$`.
   - Ensure there are **no extra blank lines** within the block.
   - Ensure there are **no extra spaces before or after** the `$$` delimiters.

   **Example Fix:**
   ```markdown
   Incorrect:
   \[
   x = y + z
   \]

   Correct:
   $$
   x = y + z
   $$
   ```

#### 🔧 Summary Fix Procedure

- Replace `\(` with `$`, and `\)` with `$` for inline math.
- Replace `\[` with `$$`, and `\]` with `$$` for block math.
- Replace any **single dollar signs used as block delimiters** (on their own line) with `$$`.
- Strip spaces **inside** `$...$` and ensure inline math is **surrounded by spaces or punctuation**.

---

This validator ensures that GitHub will render all LaTeX equations correctly by using standard `$` and `$$` delimiters for inline and block equations, respectively, **preventing formatting artifacts such as extra spaces inside math blocks or improper spacing around inline expressions**.
