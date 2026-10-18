"""Compatibility entry point for the shared G-code preparation pipeline."""
import sys

from simulation_main import main


if __name__ == "__main__":
    main(["--mode", "prepare", *sys.argv[1:]])
