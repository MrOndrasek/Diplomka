"""PostgreSQL robot registry; schema is managed exclusively by schema.py."""
from measurement_system.drivers.db_setup.schema import ensure_measurement_tables


def ensure_robot_schema(conn):
    ensure_measurement_tables(conn)


def list_robots(conn):
    ensure_robot_schema(conn)
    with conn.cursor() as cur:
        cur.execute("""SELECT id, robot_name, COALESCE(robot_type, robot_name),
                               COALESCE(host, ''), COALESCE(port, 30001), is_active
                        FROM robots ORDER BY id""")
        rows = cur.fetchall()
    active = next((r[1] for r in rows if r[5]), rows[0][1] if rows else "")
    return {
        "robots": [
            {"id": int(r[0]), "name": r[1], "type": r[2], "host": r[3], "port": int(r[4])}
            for r in rows
        ],
        "selected_robot": active,
    }


def add_robot(conn, name, host="172.31.1.147", port=30001, robot_type=None):
    name = str(name).strip()
    host = str(host).strip()
    port = int(port)
    if not name or not host or not 1 <= port <= 65535:
        raise ValueError("Robot name, host and valid TCP port are required")
    ensure_robot_schema(conn)
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT id FROM robots WHERE robot_name = %s", (name,))
            if cur.fetchone():
                raise ValueError(f"Robot '{name}' already exists")
            cur.execute("""INSERT INTO robots
                (robot_name, robot_type, host, port, is_active)
                VALUES (%s, %s, %s, %s, FALSE) RETURNING id""",
                (name, robot_type or name, host, port))
            robot_id = int(cur.fetchone()[0])
        conn.commit()
        return robot_id
    except Exception:
        conn.rollback()
        raise


def update_robot(conn, name, host, port, robot_type=None, select=True):
    ensure_robot_schema(conn)
    try:
        with conn.cursor() as cur:
            cur.execute("""UPDATE robots SET host=%s, port=%s,
                         robot_type=COALESCE(%s, robot_type), is_active=%s
                         WHERE robot_name=%s RETURNING id""",
                        (str(host).strip(), int(port), robot_type, bool(select), name))
            row = cur.fetchone()
            if row is None:
                raise LookupError(f"Robot '{name}' not found in PostgreSQL")
            if select:
                cur.execute("UPDATE robots SET is_active=FALSE WHERE robot_name<>%s", (name,))
        conn.commit()
        return int(row[0])
    except Exception:
        conn.rollback()
        raise


def select_robot(conn, name):
    ensure_robot_schema(conn)
    try:
        with conn.cursor() as cur:
            cur.execute("UPDATE robots SET is_active=(robot_name=%s)", (name,))
            if cur.rowcount == 0:
                raise LookupError(f"Robot '{name}' not found in PostgreSQL")
            cur.execute("SELECT id FROM robots WHERE robot_name=%s", (name,))
            row = cur.fetchone()
            if row is None:
                raise LookupError(f"Robot '{name}' not found in PostgreSQL")
        conn.commit()
    except Exception:
        conn.rollback()
        raise


def get_selected_robot(conn):
    data = list_robots(conn)
    name = data["selected_robot"]
    return next((r for r in data["robots"] if r["name"] == name), None)


def sync_robot_names(conn, names):
    """Compatibility helper: ensure legacy names exist, without local profiles."""
    ensure_robot_schema(conn)
    result = {}
    try:
        with conn.cursor() as cur:
            for raw in names:
                name = str(raw).strip()
                if not name:
                    continue
                cur.execute("SELECT id FROM robots WHERE robot_name=%s", (name,))
                row = cur.fetchone()
                if row is None:
                    cur.execute("INSERT INTO robots(robot_name, robot_type) VALUES (%s,%s) RETURNING id", (name, name))
                    row = cur.fetchone()
                result[name] = int(row[0])
        conn.commit()
        return result
    except Exception:
        conn.rollback()
        raise
