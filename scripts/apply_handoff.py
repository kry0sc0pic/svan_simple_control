#!/usr/bin/env python3
"""
apply_handoff.py
----------------
Applies the state-sync patch from handoff.txt into the SVAN joystick commander
source file. Run this once on the hardware after cloning/updating the repo.

Usage:
    python3 scripts/apply_handoff.py

Configuration:
    Set TARGET_FILE below to the absolute path of the joystick commander
    Python source file on your hardware before running.
"""

import ast
import shutil
import sys
import os

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURE THIS PATH before running on the hardware.
# Example: "/home/svan/dev/xMo/src/svan_joystick_commander/src/joystick_commander.py"
TARGET_FILE = "/path/to/joystick_commander.py"
# ─────────────────────────────────────────────────────────────────────────────

PATCH_FILE = os.path.join(os.path.dirname(__file__), "..", "handoff.txt")

# A unique string that only exists in the patch — used to detect if it has
# already been applied.
IDEMPOTENCY_MARKER = "state_sync_callback"


def find_insertion_line(source: str) -> int:
    """
    Use the AST to find a safe insertion point.

    Strategy (in priority order):
      1. The line just before the first call to rospy.spin() at the top level.
      2. The line just after the last top-level import statement.
      3. The very end of the file (fallback).

    Returns the 0-based line index at which the patch should be inserted.
    """
    lines = source.splitlines()
    tree = ast.parse(source)

    # --- 1. Find rospy.spin() call ---
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Expr)
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Attribute)
            and node.value.func.attr == "spin"
        ):
            # node.lineno is 1-based; insert one line before it
            return node.lineno - 1

    # --- 2. Find last top-level import ---
    last_import_line = 0
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            last_import_line = node.lineno  # 1-based

    if last_import_line:
        return last_import_line  # insert after this line (0-based = lineno)

    # --- 3. Fallback: end of file ---
    return len(lines)


def main():
    # ── Validate target file ──────────────────────────────────────────────────
    if TARGET_FILE == "/path/to/joystick_commander.py":
        print(
            "ERROR: TARGET_FILE has not been configured.\n"
            "       Open scripts/apply_handoff.py and set TARGET_FILE to the\n"
            "       absolute path of the joystick commander source file."
        )
        sys.exit(1)

    if not os.path.isfile(TARGET_FILE):
        print(f"ERROR: Target file not found: {TARGET_FILE}")
        sys.exit(1)

    # ── Read files ────────────────────────────────────────────────────────────
    patch_path = os.path.abspath(PATCH_FILE)
    if not os.path.isfile(patch_path):
        print(f"ERROR: Patch file not found: {patch_path}")
        sys.exit(1)

    with open(TARGET_FILE, "r") as f:
        original_source = f.read()

    with open(patch_path, "r") as f:
        patch_code = f.read().rstrip("\n")

    # ── Idempotency check ─────────────────────────────────────────────────────
    if IDEMPOTENCY_MARKER in original_source:
        print("INFO: Patch has already been applied (idempotency marker found).")
        print(f"      Target: {TARGET_FILE}")
        sys.exit(0)

    # ── Validate patch is parseable Python ───────────────────────────────────
    try:
        ast.parse(patch_code)
    except SyntaxError as e:
        print(f"ERROR: handoff.txt contains a syntax error: {e}")
        sys.exit(1)

    # ── Find insertion point ──────────────────────────────────────────────────
    try:
        insertion_line = find_insertion_line(original_source)
    except SyntaxError as e:
        print(
            f"ERROR: Target file contains a syntax error and could not be parsed: {e}"
        )
        sys.exit(1)

    # ── Build patched source ──────────────────────────────────────────────────
    lines = original_source.splitlines(keepends=True)

    # Ensure the patch is surrounded by blank lines for readability
    patch_block = "\n\n" + patch_code + "\n\n"

    patched_lines = lines[:insertion_line] + [patch_block] + lines[insertion_line:]
    patched_source = "".join(patched_lines)

    # ── Validate the result parses cleanly ───────────────────────────────────
    try:
        ast.parse(patched_source)
    except SyntaxError as e:
        print(
            f"ERROR: Patched result has a syntax error — aborting without writing: {e}"
        )
        sys.exit(1)

    # ── Back up original ──────────────────────────────────────────────────────
    backup_path = TARGET_FILE + ".bak"
    shutil.copy2(TARGET_FILE, backup_path)
    print(f"INFO: Backup saved to {backup_path}")

    # ── Write patched file ────────────────────────────────────────────────────
    with open(TARGET_FILE, "w") as f:
        f.write(patched_source)

    print(f"SUCCESS: Patch applied to {TARGET_FILE}")
    print(f"         Inserted at line {insertion_line + 1}.")
    print("         Restart the joystick commander node for changes to take effect.")


if __name__ == "__main__":
    main()
