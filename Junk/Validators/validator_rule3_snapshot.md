
## Markdown Equation Block Validator – Rule 3 (v2.0)

This document defines **Rule 3** of the Markdown LaTeX validator system. Rule 3 addresses a known rendering bug in GitHub-flavored Markdown that affects repeated use of `\mathbf{}` with subscripts in a single math block.

---

### ❌ Rule 3: Avoid Multiple Uses of `\mathbf{e}_{ij}` in the Same Math Block

**Issue:**
The following pattern renders correctly **once**, but subsequent uses in the same LaTeX block fail:
```latex
\mathbf{e}_{ij} \quad \mathbf{e}_{ij}  ← 🚫 renders incorrectly on second use
```

This appears to be a GitHub rendering bug, not intended behavior.

---

### ✅ Approved Workaround:
Use an escaped underscore to bypass the issue:
```latex
\mathbf{e} \_{ij} \quad \mathbf{e} \_{ij}
```
GitHub treats this form as valid even when repeated in the same block. It renders the subscript correctly without triggering the bug.

---

### 🔍 Detection Pattern (Regex):
Match the known signature that causes the bug:
```
\\mathbf\{e\}_\{ij\}
```

---

### ✅ Summary:
- Use `\mathbf{e} \_{ij}` instead of `\mathbf{e}_{ij}`
- This workaround is safe for **multiple uses** in a single equation block

---

**Status:** This rule targets a GitHub Markdown rendering bug. The workaround avoids triggering the failure and provides consistent rendering.
