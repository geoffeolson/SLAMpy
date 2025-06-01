## Markdown Equation Block Validator (v1.0)

This document defines the formatting rules used to validate LaTeX math blocks in Markdown files intended for GitHub rendering.

### ✅ Rule 1: No `\(...\)` or `\[...\]` Delimiters
**Description:** Do not use LaTeX-style delimiters like `\(...\)` or `\[...\]` for math.

**Reason:** GitHub-flavored Markdown does not support these delimiters. Instead:
- Use `$...$` for inline math
- Use `$$...$$` for block math (with blank lines above and below)

**Enforcement:**
- Replace all `\( math \)` with `$ math $`
- Replace all `\[ math \]` with `$$ math $$`
- Ensure `$$...$$` blocks are properly surrounded by blank lines (see Rule 2 — not yet active)

**Example (invalid):**
```markdown
This equation includes inline math like \( H \) and \( b \).
This block uses: \[ E = mc^2 \]
```

**Example (valid):**
```markdown
Inline: $H$ and $b$

Block:

$$
E = mc^2
$$
```

**Status:** Implemented and tested in the "Validator Test – Rule 1" document.

---

*Additional rules will be added incrementally after Rule 1 is confirmed stable.*
