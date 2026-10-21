"""Validate optional, fixed first jobs without changing collision checks."""


def validate_initial_assignments(printer, assignments):
    assignments = dict(assignments or {})
    if len(set(assignments.values())) != len(assignments):
        raise ValueError("Initial cells must be different for each printer")
    for p, candidate in assignments.items():
        if p not in printer or candidate not in printer[p].codes:
            raise ValueError(f"Initial cell {candidate} is not reachable by printer {p}")
        if printer[p].codes[candidate][0] != "A":
            raise ValueError(f"Initial cell {candidate} is not available for printer {p}")
    return assignments
