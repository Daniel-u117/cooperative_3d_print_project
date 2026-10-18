"""Default locations, independent of the shell's current directory."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
INPUT_DIR = PROJECT_ROOT / "ulti_maker"
OUTPUT_DIR = PROJECT_ROOT / "output"
