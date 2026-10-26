"""Resolve one explicit saved-map path; never fall back after a bad selection."""
import os
from pathlib import Path


def resolve_map_yaml(value):
    """Return an existing absolute map-YAML path, expanding ~ and environment vars."""
    raw = os.fspath(value)
    if not raw.strip():
        raise ValueError("Select a saved map YAML file; the map path cannot be empty")
    path = Path(os.path.expandvars(raw)).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Selected map YAML does not exist: {path}")
    return str(path)
