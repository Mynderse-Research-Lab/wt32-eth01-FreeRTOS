import sys
import re

with open(r'docs\PICK_SCHEDULER_KINEMATICS.md', 'r', encoding='utf-8') as f:
    text = f.read()

# Collapse multiple blank lines into single blank lines
text = re.sub(r'\n{3,}', '\n\n', text)

with open(r'docs\PICK_SCHEDULER_KINEMATICS.md', 'w', encoding='utf-8') as f:
    f.write(text)
