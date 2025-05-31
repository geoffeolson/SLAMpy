# LaTeX Equations

## Using ChatGPT with LaTeX Equations

We follow a structured workflow when working with documents containing LaTeX equations:

1. **Edit** all documents with LaTeX equations inside ChatGPT using a format compatible with ChatGPT.
2. **Export** the document for external use (e.g., GitHub) using a link to download a version reformatted to the appropriate Markdown style.
3. **Import** external Markdown documents by uploading them to ChatGPT, where they will be automatically reformatted to display properly within ChatGPT.
4. This document defines the rules we follow to support this workflow.

---

## GitHub Markdown Equations

GitHub uses a limited subset of LaTeX, rendered through its internal KaTeX engine. This imposes constraints on environments, macros, and placement of equations. Differences in behavior may occur compared to other renderers such as MathJax or ChatGPT's internal LaTeX system.

### Rules for Formatting Equations

This section outlines best practices and limitations for writing LaTeX equations in GitHub-flavored Markdown. Following these rules ensures consistent rendering across platforms and avoids common formatting issues.

### 1. Use Double Dollar Signs for Display Equations

Use `$$...$$` to denote display math blocks.

```markdown
$$
H = J^\top \Omega J
$$
```

Avoid `\[ ... \]` or `\begin{equation} ... \end{equation}` as they may not render on GitHub.

---

### 2. Use Single Dollar Signs for Inline Math

Use `$...$` for short expressions embedded in text.

Example:

```markdown
The matrix $H$ is symmetric.
```

---

### 3. Avoid LaTeX Environments (align, eqnarray)

GitHub does not support `\begin{align}`, `\begin{eqnarray}`, etc. Use plain `$$...$$` blocks instead.

---

### 4. Do Not Use `\displaystyle`

GitHub already renders equations in display style. `\displaystyle` is unnecessary and may cause issues.

---

### 5. Escape Underscores in Subscripts if Needed

Underscores `_` may break formatting outside of LaTeX blocks. Inside LaTeX, `$x_i$` is fine. Outside LaTeX, use backticks or escape the underscore.

```markdown
File name: `jacobian_A_ij.txt`
```

---

### 6. Use ASCII for Greek Letters When Necessary

GitHub does not allow direct input of Unicode Greek letters inside LaTeX. Use LaTeX-style names like `\mu`, `\theta`, etc.

```markdown
$$
\mu = \frac{1}{n} \sum x_i
$$
```

---

### 7. LaTeX in Lists (merged with Rule 10)

**Avoid using LaTeX block equations (`$$...$$`) inside bullet or numbered lists.** These often fail to render properly on GitHub and may break the list structure.

**Guidelines for compatibility:**

1. Use **inline LaTeX** with **single dollar signs** (`$...$`) when writing math in bullet or numbered list items.
2. If you must use multiple equations or want better formatting:

   * Place the equations on a **new line**, and
   * **Remove the bullet/number formatting** from that line (treat it as normal text).
3. Avoid using `\quad`, `\qquad`, or comma-separated equations inside `$$...$$` blocks in any context.
4. In regular (non-list) sections, use one equation per `$$...$$` block to improve readability.

**Examples:**

✅ Inline equation in bullet list:

```markdown
- Update the poses using $x \leftarrow x + \delta$.
```

✅ Block equation outside list:

```markdown
$$
x \leftarrow x + \delta
$$
```

❌ Do **not** do this in a list:

```markdown
- Update the poses:

  $$
  x \leftarrow x + \delta
  $$
```

---

### 8. No `\newcommand` or Custom Macros

Custom commands are not supported on GitHub. Use full LaTeX expressions.

---

### 9. Use Proper Matrix Formatting

Use `\begin{bmatrix} ... \end{bmatrix}` for matrix formatting.

```markdown
$$
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$
```

Do not separate matrices with commas or stack them in one block. Use separate blocks or describe them in text.

---

### 10. Avoid Backslash Escape Sequences in Text Files

Some tools or editors (including ChatGPT export pipelines) may **misinterpret escape sequences** like `\t`, `\n`, `\b`, or `\f`, converting them into control characters.

**Guidelines:**

* Avoid using editors (like Notepad++) that treat `\` escape sequences as special unless properly configured.
* Use editors like Visual Studio Code, or always check files after saving.
* Always review exported Markdown documents for corrupted LaTeX syntax.

**Common corruptions to search and fix:**

* `\b egin{bmatrix}` → should be `\begin{bmatrix}`
* `\f rac{}` → should be `\frac{}`
* `\t imes` → should be `\times`
* `\a lpha` → should be `\alpha`
* `\a rray` → should be `\array`

These may result from `\b`, `\f`, `\t`, `\a`, etc., being interpreted as control characters. Use search-and-replace to correct them.

**Important Note:**
In some cases, ChatGPT may internally convert LaTeX escape sequences during **download or file creation**, even if the file is never opened or edited. Always check any file exported from ChatGPT.

---

### 11. Line Endings and Indentation

Avoid inserting hard line breaks inside equations. If you need a line break inside display math, use `\\`.

```markdown
$$
A = \begin{bmatrix}
a & b \\
c & d
\end{bmatrix}
$$
```

Do not indent `$$...$$` blocks unless absolutely necessary.

---

*Last updated: May 2025*

## ChatGPT Equations

ChatGPT uses its own LaTeX renderer which differs from GitHub's KaTeX in a number of ways. It may support broader syntax and more forgiving formatting but can behave unpredictably in edge cases. Rendering quality and compatibility can change depending on platform updates.

### Rules for Formatting Equations

This section outlines the rules for importing LaTeX documents into ChatGPT and formatting them for correct rendering:

1. Use inline math (`$...$`) for expressions in lists.
2. Avoid using `$$...$$` blocks inside bullet or numbered lists.
3. Equations that must span multiple lines or require block formatting should be placed outside list formatting.
4. Always check files exported from ChatGPT for conversion errors, especially escape sequences like `\b`, `\t`, `\f`, etc.
5. Use Visual Studio Code or similar tools to inspect and correct control character corruption.
6. Complex LaTeX should be previewed and validated before import.
7. Avoid using GitHub-specific hacks (like KaTeX-only macros) inside ChatGPT.

*(This section will be expanded in future updates.)*
