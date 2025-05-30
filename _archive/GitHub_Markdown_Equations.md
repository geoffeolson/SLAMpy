# GitHub Markdown Equations

This document lists formatting rules and best practices for writing LaTeX equations in GitHub-flavored Markdown (`.md`) files. These guidelines are derived from experimentation and known GitHub rendering behavior.

---

## ✅ Confirmed LaTeX Formatting Rules

1. **Use Single Dollar Signs for Inline Math**  
   Inline equations should use `$...$`.  
   Example:  
   `$E = mc^2$`

2. **Use Double Dollar Signs for Block Math**  
   Block equations should use `$$...$$`, with no indentation.  
   Example:  
   ```markdown
   $$
   H = J^T \Omega J
   $$
   ```

3. **Avoid Extra Newlines Inside Math Blocks**  
   Do not insert a blank line before or after `$$`. This may break rendering.  
   ❌ Incorrect:
   ```markdown
   $$

   x^2 + y^2 = z^2

   $$
   ```

   ✅ Correct:
   ```markdown
   $$
   x^2 + y^2 = z^2
   $$
   ```

4. **No LaTeX Environments (e.g., `\begin{bmatrix}`)**  
   GitHub Markdown does **not** support LaTeX environments like `bmatrix`, `align`, or `cases`. Use simple inline notation instead.  
   ❌ `\begin{bmatrix} ... \end{bmatrix}`  
   ✅ Use manual formatting like `\left[ \begin{array}{cc} ... \end{array} \right]` or break into separate lines with plain formatting.

5. **Avoid Backslashes Before Non-LaTeX Commands**  
   GitHub interprets sequences like `\b`, `\f`, `\t` as escape characters. These should be written carefully or avoided in `.md` files.  
   ✅ Use raw string literals or code blocks to prevent escaping, or replace `\t` with `\\t` in literal strings.

6. **Avoid Using Multiple Subscripts with `\mathbf{}`**  
   GitHub LaTeX rendering often breaks when using multiple subscripts in bolded symbols.  
   ❌ `\mathbf{x}_{ij}` (works only once per block, breaks after that)  
   ✅ Prefer `x_{ij}` or drop bolding unless absolutely needed.

7. **Do Not Mix Bullet Lists and Block Math Without Care**  
   When placing block math inside a list item, ensure it is indented properly and doesn't conflict with the list structure. See proposed rule 10.

8. **Use UTF-8 Compatible Editors**  
   Avoid editors (like Notepad++) that automatically convert backslashes or escape sequences, which may corrupt LaTeX. Use Visual Studio Code, Sublime Text, or GitHub’s web editor instead.

9. **Test Your Markdown in GitHub Directly**  
   GitHub's LaTeX rendering differs from local editors. Always test `.md` rendering by previewing on GitHub itself.

---

## 🔄 Proposed Rule 10: Equations Inside Bullet Lists

When using LaTeX block equations (`$$...$$`) inside a bullet list, **GitHub will not render them properly** unless the math block is treated as part of the list item’s content. Here are a few strategies:

### ❌ Don't Do This:

```markdown
- Step 3:
$$
H = J^T \Omega J
$$
```

GitHub often breaks rendering in this structure.

### ✅ Recommended:

- Step 3: Use the system matrix and right-hand vector:

  $$
  H = J^T \Omega J, \quad b = J^T \Omega e
  $$

- Keep the math block indented and part of the same list level as the bullet text.

- Alternatively, **inline** short equations using single `$` delimiters:
  - Step 3: Use the formula `$H = J^T \Omega J$` to build the system.
