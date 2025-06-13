## Latex Validator

This validator ensures that LaTeX equations in a Markdown file use for GitHub will properly rendor equations.

---

### Rule 1: Inline LaTeX Deliminators

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

