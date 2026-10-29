"""Per-user PostgreSQL settings, stored outside source control."""
import json
import os
from pathlib import Path

SETTINGS_FILE = Path.home() / '.config' / 'diplomka' / 'postgresql.json'
FIELDS = ('host', 'port', 'dbname', 'user', 'password')


def load_settings():
    if not SETTINGS_FILE.exists():
        return {}
    data = json.loads(SETTINGS_FILE.read_text(encoding='utf-8'))
    return {k: data[k] for k in FIELDS if k in data}


def save_settings(values):
    SETTINGS_FILE.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(SETTINGS_FILE.parent, 0o700)
    content = {k: values[k] for k in FIELDS if k in values}
    temporary = SETTINGS_FILE.with_suffix('.tmp')
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8') as stream:
        json.dump(content, stream, ensure_ascii=False, indent=2)
    os.replace(temporary, SETTINGS_FILE)
    os.chmod(SETTINGS_FILE, 0o600)


def connect_with_settings(values=None):
    import psycopg2
    values = load_settings() if values is None else values
    missing = [name for name in ('host', 'dbname', 'user') if not values.get(name)]
    if missing:
        raise RuntimeError('Nastav PostgreSQL v UI: ' + ', '.join(missing))
    kwargs = {k: values[k] for k in FIELDS if values.get(k) not in (None, '')}
    kwargs['connect_timeout'] = 4
    return psycopg2.connect(**kwargs)
