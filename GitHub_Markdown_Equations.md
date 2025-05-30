# GitHub Markdown Equations

This document collects formatting rules, examples, and best practices for writing LaTeX equations in GitHub-flavored Markdown (GFM). These guidelines are derived from both experimentation and known GitHub rendering behavior.

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

4. **LaTeX Environments (e.g., `\begin{bmatrix}`)**  
   GitHub Markdown **partially** supports LaTeX environments like `bmatrix` as of 2022. However, use caution: rendering may fail in some contexts or Markdown parsers.  
   ✅ This works in GitHub:
   ```markdown
   $$
   \mathbf{x}_i = \begin{bmatrix} x_i \\ y_i \\ \theta_i \end{bmatrix}
   $$
   ```

5. **Avoid Backslashes Before Non-LaTeX Commands**  
   GitHub interprets sequences like `\b`, `\f`, `\t` as escape characters. These should be written carefully or avoided in `.md` files.  
   ✅ Use raw string literals or code blocks to prevent escaping, or replace `\t` with `\\t` in literal strings.

6. **Avoid Using Multiple Subscripts with `\mathbf{}`**  
   GitHub LaTeX rendering may break when using multiple subscripts in bolded symbols.  
   ❌ `\mathbf{x}_{ij}` may fail after one instance.  
   ✅ Prefer `x_{ij}` or drop bolding unless absolutely needed.

7. **Do Not Mix Bullet Lists and Block Math Without Care**  
   When placing block math inside a list item, ensure it is indented properly and doesn't conflict with the list structure.

8. **Use UTF-8 Compatible Editors**  
   Avoid editors (like Notepad++) that automatically convert backslashes or escape sequences. Use Visual Studio Code, Sublime Text, or GitHub’s web editor instead.

9. **Test Your Markdown in GitHub Directly**  
   GitHub's LaTeX rendering differs from local editors. Always preview `.md` files on GitHub.

10. **Equations Inside Bullet Lists**  
    Block math (`$$...$$`) inside a bullet list must be indented to be part of the list item’s content, or it may not render correctly.  
    ❌ Incorrect:
    ```markdown
    - Step 3:
    $$
    H = J^T \Omega J
    $$
    ```
    ✅ Correct:
    ```markdown
    - Step 3: Use the system matrix and right-hand vector:

      $$
      H = J^T \Omega J, \quad b = J^T \Omega e
      $$
    ```
    - Alternatively, use inline math for short expressions:  
      `- Step 3: Use $H = J^T \Omega J$`.

11. **Spacing and Inline Formatting**  
    Use `\quad` or `\,` for spacing within math.  
    ✅ `\,` is a valid LaTeX command for thin spacing and works correctly on GitHub.
    Use `\text{}` for regular text inside equations.

12. **Matrix Formatting**  
    Avoid placing commas between matrices directly unless they are inside the same equation block **and properly grouped**. GitHub may fail to render if a comma is used next to line breaks (e.g. `\\`).  
    ❌ Problematic:
    ```markdown
    $$
    \begin{bmatrix} x_i \\ y_i \end{bmatrix}, \quad \begin{bmatrix} x_j \\ y_j \end{bmatrix}
    $$
    ```
    GitHub might not render this due to misinterpretation of the comma and line breaks.

    ✅ Recommended Fixes:
    - **Avoid the comma:**
    ```markdown
    $$
    \begin{bmatrix} x_i \\ y_i \end{bmatrix} \quad \begin{bmatrix} x_j \\ y_j \end{bmatrix}
    $$
    ```

    - **Wrap each matrix in grouping braces:**
    ```markdown
    $$
    {\begin{bmatrix} x_i \\ y_i \end{bmatrix}}, \quad {\begin{bmatrix} x_j \\ y_j \end{bmatrix}}
    $$
    ```

    - **Stack vertically instead:**
    ```markdown
    $$
    \mathbf{x}_i = \begin{bmatrix} x_i \\ y_i \end{bmatrix} \\
    \mathbf{x}_j = \begin{bmatrix} x_j \\ y_j \end{bmatrix}
    $$
    ```

13. **Troubleshooting**  
    If math does not render on GitHub, check for typos, incorrect escapes, or mismatched delimiters.  
    Use a local Markdown editor with KaTeX or MathJax support (e.g., Typora, Obsidian, VS Code with Markdown Preview Enhanced) to test locally.

---

## 🔗 Resources

- [GitHub Math Support](https://github.blog/changelog/2022-05-19-markdown-support-for-math-expressions/)
- [KaTeX Documentation](https://katex.org/docs/supported.html)
- [MathJax Documentation](https://docs.mathjax.org/)