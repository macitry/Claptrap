"""Load project-wide runtime configuration from src/config/config.json."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = Path(__file__).resolve().parent / "config.json"


def load_app_config() -> dict[str, Any]:
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def config_section(*keys: str) -> dict[str, Any]:
    section: Any = load_app_config()
    for key in keys:
        try:
            section = section[key]
        except KeyError as exc:
            dotted_key = ".".join(keys)
            raise KeyError(f"Missing config section: {dotted_key}") from exc

    if not isinstance(section, dict):
        dotted_key = ".".join(keys)
        raise TypeError(f"Config section must be an object: {dotted_key}")
    return section


def resolve_project_path(value: str | None) -> Path | None:
    if value is None:
        return None

    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return (PROJECT_ROOT / path).resolve()


def require_project_path(section: dict[str, Any], key: str) -> Path:
    value = section.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"Config value must be a non-empty path string: {key}")

    resolved = resolve_project_path(value)
    if resolved is None:
        raise ValueError(f"Config value must be a path string: {key}")
    return resolved
