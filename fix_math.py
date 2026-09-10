import sys

with open(r'docs\PICK_SCHEDULER_KINEMATICS.md', 'r', encoding='utf-8') as f:
    text = f.read()

# 1. Fix the underscore in \text{epoch_us}
text = text.replace(r'$t_{\text{epoch_us}}$', r'$t_{\text{epoch\_us}}$')

# 2. Add blank lines before and after block math
# First, let's find all occurrences of '\n$$ ' or ' $$\n' and format them properly
# Actually, the file currently has:
# $$ L_{\mathrm{cam}... $$
# Let's replace '$$' blocks with proper spacing.
import re

# Find lines that start with $$ and end with $$
# We'll just replace '$$ ... $$' with '\n$$\n...\n$$\n'
def fix_block_math(match):
    content = match.group(1).strip()
    return f"\n\n$$\n{content}\n$$\n\n"

text = re.sub(r'(?m)^[ \t]*\$\$(.*?)\$\$[ \t]*$', fix_block_math, text)

# 3. For the italic line: *(e.g., at $v \approx 1500 \text{ mm/s}$ ... )*
# Let's remove the * italics and just leave it as normal text, or use HTML <em>
# Actually, GitHub math should work inside italics, but maybe it's the fact that 
# it's on a single line that starts with `*`. A line starting with `*` is a bullet point!
# Oh!! `*(e.g.,` -> `*` followed by `(` is NOT a bullet point, it's just italics if there is a closing `*`.
# Let's just remove the `*` and use `_` or nothing. Let's just remove the `*`.
text = text.replace(r'*(e.g., at $v \approx 1500 \text{ mm/s}$ with $D = 679.45 \text{ mm}$, $\tau \approx 0.45 \text{ s}$)*',
                    r'(e.g., at $v \approx 1500 \text{ mm/s}$ with $D = 679.45 \text{ mm}$, $\tau \approx 0.45 \text{ s}$)')

# Also, check if there are any other block maths that were missed
text = re.sub(r'([^\n])\n\$\$', r'\1\n\n$$', text)
text = re.sub(r'\$\$\n([^\n])', r'$$\n\n\1', text)

with open(r'docs\PICK_SCHEDULER_KINEMATICS.md', 'w', encoding='utf-8') as f:
    f.write(text)
