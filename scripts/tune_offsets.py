#!/usr/bin/env python3
"""
tune_offsets.py
---------------
Interactive terminal tool to tune the velocity drift-correction offsets
written to  config/hardware_offsets.yaml  and read by  src/hardware.py.

Controls:
    Up / Down arrow   — increase / decrease  offset_velocity_y  (longitudinal)
    Left / Right arrow — decrease / increase  offset_velocity_x  (lateral)
    r                 — reset both offsets to 0.0
    s                 — save and keep tuning
    q / Ctrl+C / Esc  — save and quit

The file is saved automatically on exit.  hardware.py re-reads it on each
startup, so no ROS rebuild is required — just restart the node.

Usage:
    python3 scripts/tune_offsets.py
"""

import os
import sys
import termios
import tty

import yaml

# ── Config file path (relative to this script's parent directory) ─────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(REPO_ROOT, "config", "hardware_offsets.yaml")

STEP = 0.001  # change per key press
MIN_VAL = -1.0
MAX_VAL = 1.0


# ── YAML helpers ──────────────────────────────────────────────────────────────


def load_offsets():
    if not os.path.exists(CONFIG_PATH):
        return {"offset_velocity_x": 0.0, "offset_velocity_y": 0.0}
    with open(CONFIG_PATH) as f:
        data = yaml.safe_load(f)
    return {
        "offset_velocity_x": float(data.get("offset_velocity_x", 0.0)),
        "offset_velocity_y": float(data.get("offset_velocity_y", 0.0)),
    }


def save_offsets(ox: float, oy: float):
    # Preserve comments by writing the file from scratch with the header block.
    content = (
        "# Hardware velocity drift-correction offsets.\n"
        "#\n"
        "# These values are added to every movement command sent by hardware.py to\n"
        "# counteract the robot's physical bias on a given surface.\n"
        "#\n"
        "# Tune interactively with:\n"
        "#   python3 scripts/tune_offsets.py\n"
        "#\n"
        "# Units: normalised velocity (-1.0 to 1.0)\n"
        "#   offset_velocity_x  — lateral axis  (positive = right)\n"
        "#   offset_velocity_y  — longitudinal axis (positive = forward)\n"
        "\n"
        f"offset_velocity_x: {ox:.4f}\n"
        f"offset_velocity_y: {oy:.4f}\n"
    )
    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    with open(CONFIG_PATH, "w") as f:
        f.write(content)


# ── Raw key reading ───────────────────────────────────────────────────────────


def get_key(fd: int) -> str:
    """Read one keypress (blocking). Returns a string token."""
    ch = os.read(fd, 1)
    if ch == b"\x1b":
        # Escape sequence — read up to 2 more bytes
        try:
            seq = os.read(fd, 2)
        except OSError:
            return "esc"
        if seq == b"[A":
            return "up"
        elif seq == b"[B":
            return "down"
        elif seq == b"[C":
            return "right"
        elif seq == b"[D":
            return "left"
        else:
            return "esc"
    elif ch in (b"q", b"Q"):
        return "quit"
    elif ch in (b"r", b"R"):
        return "reset"
    elif ch in (b"s", b"S"):
        return "save"
    elif ch == b"\x03":  # Ctrl+C
        return "quit"
    return "other"


# ── TUI rendering ─────────────────────────────────────────────────────────────

CLEAR = "\033[2J\033[H"
BOLD = "\033[1m"
DIM = "\033[2m"
GREEN = "\033[32m"
CYAN = "\033[36m"
RESET = "\033[0m"

BAR_WIDTH = 40  # characters for the bar


def make_bar(value: float) -> str:
    """Render a [-1 ──── 0 ──── +1] bar with a marker at value."""
    centre = BAR_WIDTH // 2
    pos = int((value + 1.0) / 2.0 * BAR_WIDTH)
    pos = max(0, min(BAR_WIDTH, pos))
    bar = ["-"] * BAR_WIDTH
    bar[centre] = "┼"
    bar[pos] = "█"
    return "[" + "".join(bar) + "]"


def render(ox: float, oy: float, message: str = ""):
    lines = [
        f"{CLEAR}{BOLD}SVAN Offset Tuner{RESET}",
        f"{DIM}config: {CONFIG_PATH}{RESET}",
        "",
        f"  {BOLD}offset_velocity_y{RESET}  (longitudinal)   {CYAN}{oy:+.4f}{RESET}",
        f"  {make_bar(oy)}",
        f"  {DIM}Up ↑ / Down ↓  to adjust   (step {STEP}){RESET}",
        "",
        f"  {BOLD}offset_velocity_x{RESET}  (lateral)        {CYAN}{ox:+.4f}{RESET}",
        f"  {make_bar(ox)}",
        f"  {DIM}Left ← / Right → to adjust  (step {STEP}){RESET}",
        "",
        f"  {DIM}r — reset to 0.0   s — save   q / Esc — save & quit{RESET}",
    ]
    if message:
        lines.append(f"\n  {GREEN}{message}{RESET}")
    sys.stdout.write("\n".join(lines))
    sys.stdout.flush()


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    offsets = load_offsets()
    ox = offsets["offset_velocity_x"]
    oy = offsets["offset_velocity_y"]

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    message = ""

    try:
        tty.setraw(fd)
        render(ox, oy)

        while True:
            key = get_key(fd)

            if key == "up":
                oy = round(min(MAX_VAL, oy + STEP), 4)
                message = ""
            elif key == "down":
                oy = round(max(MIN_VAL, oy - STEP), 4)
                message = ""
            elif key == "right":
                ox = round(min(MAX_VAL, ox + STEP), 4)
                message = ""
            elif key == "left":
                ox = round(max(MIN_VAL, ox - STEP), 4)
                message = ""
            elif key == "reset":
                ox = 0.0
                oy = 0.0
                message = "Reset to 0.0"
            elif key == "save":
                save_offsets(ox, oy)
                message = "Saved."
            elif key == "quit":
                break

            render(ox, oy, message)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        save_offsets(ox, oy)
        sys.stdout.write(
            f"\n\n{GREEN}Saved:{RESET}  offset_velocity_x={ox:+.4f}  offset_velocity_y={oy:+.4f}\n"
        )
        sys.stdout.write(f"File: {CONFIG_PATH}\n\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
