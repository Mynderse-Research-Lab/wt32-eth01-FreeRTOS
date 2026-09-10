import sys
import re

with open(r'docs\PICK_SCHEDULER_KINEMATICS.md', 'r', encoding='utf-8') as f:
    text = f.read()

# Fix the spacing inside the math blocks
def fix_math_block_inner(match):
    content = match.group(1).strip()
    return f"$$\n{content}\n$$"

text = re.sub(r'\$\$\s+(.*?)\s+\$\$', fix_math_block_inner, text, flags=re.DOTALL)

with open(r'docs\PICK_SCHEDULER_KINEMATICS.md', 'w', encoding='utf-8') as f:
    f.write(text)
