"""Persistent, per-preparation job identity for RepRapFirmware."""
from pathlib import Path
import uuid

START_VARIABLE = "ambotJobStarted"
DONE_VARIABLE = "ambotJobDone"


def instrument_job(path):
    """Wrap a freshly generated job; return its unique completion token.

    Global variables are read through rr_model, not the consumable rr_reply queue.
    A token is unique to this printer, file variant, and preparation run.
    """
    path = Path(path)
    token = uuid.uuid4().hex
    header = (
        f"if !exists(global.{START_VARIABLE})\n"
        f'  global {START_VARIABLE} = ""\n'
        f"if !exists(global.{DONE_VARIABLE})\n"
        f'  global {DONE_VARIABLE} = ""\n'
        f'set global.{START_VARIABLE} = "{token}"\n'
    )
    footer = f'\nM400\nset global.{DONE_VARIABLE} = "{token}"\n'
    path.write_text(header + path.read_text(encoding="utf-8") + footer, encoding="utf-8")
    return token
