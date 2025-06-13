
## Markdown Equation Block Validator – Rule 1 (v1.0)

This document defines **Rule 1** of the Markdown LaTeX validator system. It validates correct usage of block-level math delimiters in GitHub-flavored Markdown.

---

### ❌ Rule 1: Do Not Use Inline Delimiters for Display Equations

**Issue:** Display-style equations must be wrapped in double-dollar signs `$$ ... $$` on their own lines. Inline math using `$...$` works only for short expressions inside a sentence.

---

### ✅ Approved Format:
```markdown
$$
A = B + C
$$
```

---

### 🔍 Detection Pattern (Regex):
Detect inline math starting and ending with a single `$` that appears alone on a line:
```
(?<!\)\$[^\$].*[^\$]\$(?!\$)
```

---

### ✅ Summary:
- Always use `$$ ... $$` for display equations
- Avoid using single `$...$` on its own line
- Use inline `$...$` only inside full sentences

---

**Status:** Rule ensures equations are block-wrapped using `$$`, which GitHub requires for proper LaTeX rendering.
