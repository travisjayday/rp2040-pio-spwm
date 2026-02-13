import os
import subprocess

Import("env")

project_dir = env.subst("$PROJECT_DIR")
out_dir = os.path.join(project_dir, "include")

items = [
    {'src': os.path.join(project_dir, "pio", "spwm.pio"), 'out': os.path.join(out_dir, "spwm.pio.h") },
    {'src': os.path.join(project_dir, "pio", "as5600.pio"), 'out': os.path.join(out_dir, "as5600.pio.h")},
]

os.makedirs(out_dir, exist_ok=True)

# Find pioasm executable
pioasm = env.WhereIs("pioasm") or env.WhereIs("pioasm.exe")
if not pioasm:
    raise RuntimeError(
        "pioasm not found in PATH. Install pico-sdk tools or add pioasm to PATH."
    )

# Run pioasm, write header
for item in items:
    with open(item['out'], "w", newline="\n") as f:
        subprocess.run([pioasm, item['src']], stdout=f, check=True)
    print(f"[pioasm] Generated {item['out']} from {item['src']}")
