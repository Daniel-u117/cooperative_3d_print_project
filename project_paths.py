"""Default locations, independent of the shell's current directory."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
INPUT_DIR = PROJECT_ROOT / "input" / "ulti_maker"
CELL_PATH_DIR = PROJECT_ROOT / "input" / "cells_spiral"
CELL_PRINT_CONFIG = PROJECT_ROOT / "config" / "cell_print.toml"
OUTPUT_DIR = PROJECT_ROOT / "output"
