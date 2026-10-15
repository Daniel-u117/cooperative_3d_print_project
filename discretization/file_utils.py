
from pathlib import Path
import re
import numpy as np



def _append_z10(lines):
    """Ensure G1 Z10 is written on its own final line."""
    if lines and not lines[-1].endswith(("\n", "\r")):
        lines[-1] += "\n"

    lines.append("G1 Z10\n")
    return lines

# def parse_beg(targ_file, folder, dest_folder=None, suffix="_b"):
#     source_path = targ_file

#     if dest_folder is None:
#         dest_folder = folder

#     dest_folder = Path(dest_folder)
#     dest_folder.mkdir(parents=True, exist_ok=True)

#     new_path = dest_folder / f"{source_path.stem}{suffix}{source_path.suffix}"

#     with open(source_path, "r", encoding="utf-8", errors="ignore") as f:
#         lines = f.readlines()

#     cut_index = None

#     # Search from the bottom upward
#     for i in range(len(lines) - 1, -1, -1):
#         if lines[i].startswith(";TIME_ELAPSED"):
#             cut_index = i
#             break

#     if cut_index is not None:
#         # Keep everything up to and including ;TIME_ELAPSED
#         lines = lines[:cut_index + 1]

#         with open(new_path, "w", encoding="utf-8") as f:
#             f.writelines(lines)



#     else:
#         # If no ;TIME_ELAPSED line exists, copy original unchanged
#         with open(new_path, "w", encoding="utf-8") as f:
#             f.writelines(lines)


def parse_beg(targ_file, folder, dest_folder=None, suffix="_b"):
    source_path = Path(targ_file)

    if dest_folder is None:
        dest_folder = folder

    dest_folder = Path(dest_folder)
    dest_folder.mkdir(parents=True, exist_ok=True)

    new_path = (
        dest_folder
        / f"{source_path.stem}{suffix}{source_path.suffix}"
    )

    with open(
        source_path,
        "r",
        encoding="utf-8",
        errors="ignore",
    ) as f:
        lines = f.readlines()

    cut_index = None

    for i in range(len(lines) - 1, -1, -1):
        if lines[i].startswith(";TIME_ELAPSED"):
            cut_index = i
            break

    if cut_index is not None:
        lines = lines[:cut_index + 1]

    lines = _append_z10(lines)

    with open(new_path, "w", encoding="utf-8") as f:
        f.writelines(lines)





def parse_end(targ_file, folder, dest_folder=None, suffix="_e"):
    source_path = targ_file


    if dest_folder is None:
        dest_folder = folder

    dest_folder = Path(dest_folder)
    dest_folder.mkdir(parents=True, exist_ok=True)

    new_path = dest_folder / f"{source_path.stem}{suffix}{source_path.suffix}"

    with open(source_path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    cut_index = None

    # Search from the bottom upward
    for i in range(len(lines)):
        if lines[i].startswith(";LAYER:0"):
            cut_index = i
            break

    if cut_index is not None:
        # Keep everything up to and including ;TIME_ELAPSED
        lines = lines[cut_index:]

        with open(new_path, "w", encoding="utf-8") as f:
            f.writelines(lines)



    else:
        # If no ;TIME_ELAPSED line exists, copy original unchanged
        with open(new_path, "w", encoding="utf-8") as f:
            f.writelines(lines)



# def parse_mid(targ_file, folder, dest_folder=None, suffix="_m"):
#     source_path = targ_file


#     if dest_folder is None:
#         dest_folder = folder

#     dest_folder = Path(dest_folder)
#     dest_folder.mkdir(parents=True, exist_ok=True)

#     new_path = dest_folder / f"{source_path.stem}{suffix}{source_path.suffix}"

#     with open(source_path, "r", encoding="utf-8", errors="ignore") as f:
#         lines = f.readlines()

#     cut_index1 = None

#     # Search from the bottom upward
#     for i in range(len(lines)):
#         if lines[i].startswith(";LAYER:0"):
#             cut_index1 = i
#             break
    
#     cut_index2 = None

#     for i in range(len(lines) - 1, -1, -1):
#         if lines[i].startswith(";TIME_ELAPSED"):
#             cut_index2 = i
#             break

#     if cut_index1 is not None and cut_index2 is not None:
#         # Keep everything up to and including ;TIME_ELAPSED
#         lines = lines[cut_index1:cut_index2+1]

#         with open(new_path, "w", encoding="utf-8") as f:
#             f.writelines(lines)



#     else:
#         # If no ;TIME_ELAPSED line exists, copy original unchanged
#         with open(new_path, "w", encoding="utf-8") as f:
#             f.writelines(lines)



def parse_mid(targ_file, folder, dest_folder=None, suffix="_m"):
    source_path = Path(targ_file)

    if dest_folder is None:
        dest_folder = folder

    dest_folder = Path(dest_folder)
    dest_folder.mkdir(parents=True, exist_ok=True)

    new_path = (
        dest_folder
        / f"{source_path.stem}{suffix}{source_path.suffix}"
    )

    with open(
        source_path,
        "r",
        encoding="utf-8",
        errors="ignore",
    ) as f:
        lines = f.readlines()

    cut_index1 = None

    for i in range(len(lines)):
        if lines[i].startswith(";LAYER:0"):
            cut_index1 = i
            break

    cut_index2 = None

    for i in range(len(lines) - 1, -1, -1):
        if lines[i].startswith(";TIME_ELAPSED"):
            cut_index2 = i
            break

    if cut_index1 is not None and cut_index2 is not None:
        lines = lines[cut_index1:cut_index2 + 1]

    lines = _append_z10(lines)

    with open(new_path, "w", encoding="utf-8") as f:
        f.writelines(lines)



