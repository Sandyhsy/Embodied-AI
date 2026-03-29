#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════════
#  SO-ARM101 MuJoCo Simulation — Quick Setup & Run Script
#  Compatible with macOS (Apple Silicon & Intel) and Linux (Ubuntu/Debian)
#  CPU-only: no GPU required
# ═══════════════════════════════════════════════════════════════════════════════

set -e

PYTHON=${PYTHON:-python3}
PIP="$PYTHON -m pip"

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   SO-ARM101 MuJoCo Simulation Setup              ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── 1. Check Python ────────────────────────────────────────────────────────────
echo "▶ Python version:"
$PYTHON --version

# ── 2. Install dependencies ────────────────────────────────────────────────────
echo ""
echo "▶ Installing Python dependencies..."
$PIP install --upgrade pip -q
$PIP install mujoco \
             "gymnasium[mujoco]" \
             matplotlib \
             numpy \
             ipywidgets \
             notebook \
             jupyterlab \
             requests \
             -q

# ── 3. Verify MuJoCo ───────────────────────────────────────────────────────────
echo ""
echo "▶ Verifying MuJoCo installation..."
$PYTHON -c "
import mujoco, numpy as np
print(f'  ✅  MuJoCo version : {mujoco.__version__}')
print(f'  ✅  numpy version  : {np.__version__}')

# Quick smoke-test: create a tiny model and step it
xml = '''<mujoco><worldbody><body><joint type=\"hinge\"/><geom type=\"sphere\" size=\"0.1\"/></body></worldbody></mujoco>'''
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
for _ in range(100): mujoco.mj_step(m, d)
print(f'  ✅  Simulation test : PASSED (100 steps, final time={d.time:.3f}s)')
"

# ── 4. What to run ─────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   Setup complete! Choose what to run:            ║"
echo "╠══════════════════════════════════════════════════╣"
echo "║                                                  ║"
echo "║  A) Jupyter notebook (Task 1.1):                 ║"
echo "║     jupyter lab 01_mujoco_basics.ipynb           ║"
echo "║                                                  ║"
echo "║  B) Interactive keyboard simulation:             ║"
echo "║     python so_arm101_keyboard_sim.py             ║"
echo "║                                                  ║"
echo "║  C) MuJoCo built-in viewer (sanity check):       ║"
echo "║     python -m mujoco.viewer                      ║"
echo "║                                                  ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

read -p "Run keyboard simulation now? (y/N) " REPLY
if [[ "$REPLY" =~ ^[Yy]$ ]]; then
    echo ""
    $PYTHON so_arm101_keyboard_sim.py
fi
