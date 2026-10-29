"""Per-user device profiles. Not stored in repository."""
import json
import os
from pathlib import Path

PATH = Path.home() / '.config' / 'diplomka' / 'devices.json'
DEFAULT = {
    'robots': [{'name': 'KUKA LBR iiwa 14 R820', 'host': '172.31.1.147', 'port': 30001}],
    'selected_robot': 'KUKA LBR iiwa 14 R820',
    'leica': {'interface': 'eno4', 'host': '127.0.0.1', 'port': 5555},
}

def load_profiles():
    result = json.loads(json.dumps(DEFAULT))
    if PATH.exists():
        saved = json.loads(PATH.read_text(encoding='utf-8'))
        for key in DEFAULT:
            if key in saved:
                result[key] = saved[key]
    return result

def save_profiles(data):
    PATH.parent.mkdir(parents=True, exist_ok=True)
    os.chmod(PATH.parent, 0o700)
    tmp = PATH.with_suffix('.tmp')
    fd = os.open(tmp, os.O_CREAT | os.O_TRUNC | os.O_WRONLY, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    os.replace(tmp, PATH)
    os.chmod(PATH, 0o600)

def selected_robot(data=None):
    d = data or load_profiles()
    return next((x for x in d['robots'] if x['name'] == d['selected_robot']), None)

def leica_endpoint(data=None):
    d = data or load_profiles()
    c = d['leica']
    return f"tcp://{c['host']}:{int(c['port'])}"
