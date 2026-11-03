"""Device accessors. Robot profiles are read exclusively from PostgreSQL.

Only Leica publisher endpoint preferences remain in a small user config file;
robot names, types, hosts, ports and active selection are never read from disk.
"""
import json
import os
from pathlib import Path

from measurement_system.drivers.db_setup.connection_settings import connect_with_settings
from measurement_system.drivers.db_setup.robot_registry import list_robots, select_robot, update_robot

_LEICA_PATH = Path.home() / ".config" / "diplomka" / "leica.json"
_LEICA_DEFAULT = {
    "interface": "eno4", "host": "127.0.0.1", "port": 5555,
    "distance_unit": "mm", "angle_unit": "rad", "vertical_convention": "zenith",
}


def _load_leica():
    result = dict(_LEICA_DEFAULT)
    try:
        if _LEICA_PATH.exists():
            saved = json.loads(_LEICA_PATH.read_text(encoding="utf-8"))
            result.update({k: saved[k] for k in _LEICA_DEFAULT if k in saved})
    except Exception:
        pass
    return result


def _save_leica(value):
    _LEICA_PATH.parent.mkdir(parents=True, exist_ok=True)
    os.chmod(_LEICA_PATH.parent, 0o700)
    tmp = _LEICA_PATH.with_suffix(".tmp")
    fd = os.open(tmp, os.O_CREAT | os.O_TRUNC | os.O_WRONLY, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(value, f, indent=2, ensure_ascii=False)
    os.replace(tmp, _LEICA_PATH)
    os.chmod(_LEICA_PATH, 0o600)


def load_profiles():
    # Keep UI launchable while PostgreSQL is offline so the user can open Settings.
    # Robot profiles are never loaded from a local fallback.
    try:
        conn = connect_with_settings()
        try:
            data = list_robots(conn)
        finally:
            conn.close()
        data["database_error"] = None
    except Exception as exc:
        data = {"robots": [], "selected_robot": "", "database_error": str(exc)}
    data["leica"] = _load_leica()
    return data


def save_profiles(data):
    """Persist selected robot in PostgreSQL and Leica endpoint separately."""
    conn = connect_with_settings()
    try:
        selected = str(data.get("selected_robot", "")).strip()
        if selected:
            select_robot(conn, selected)
    finally:
        conn.close()
    if "leica" in data:
        _save_leica(data["leica"])


def selected_robot():
    conn = connect_with_settings()
    try:
        data = list_robots(conn)
        name = data["selected_robot"]
        item = next((x for x in data["robots"] if x["name"] == name), None)
        if item is None:
            raise RuntimeError("No robot profile exists in PostgreSQL. Add a robot in Settings first.")
        return item
    finally:
        conn.close()


def leica_endpoint(data=None):
    c = (data or {}).get("leica") or _load_leica()
    return f"tcp://{c['host']}:{int(c['port'])}"


def leica_measurement_settings():
    """Return units/conventions for spherical -> Cartesian conversion."""
    return _load_leica()
