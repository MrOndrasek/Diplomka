"""Idempotent creation of the original db_setup/databaze.py schema.

Only missing tables are created. Existing tables and measurements are not deleted.
"""


def ensure_measurement_tables(conn):
    """Create the 3 core tables, then commit; rollback on failure."""
    try:
        with conn.cursor() as cur:
            cur.execute("""CREATE TABLE IF NOT EXISTS robots (
                id SERIAL PRIMARY KEY,
                robot_name TEXT NOT NULL
            )""")
            cur.execute("""CREATE TABLE IF NOT EXISTS measurements (
                id SERIAL PRIMARY KEY,
                robot_id INTEGER REFERENCES robots(id),
                time_of_measuring TIMESTAMP DEFAULT NOW(),
                name TEXT NOT NULL
            )""")
            cur.execute("""CREATE TABLE IF NOT EXISTS measurement_data (
                id SERIAL PRIMARY KEY,
                measurement_id INTEGER REFERENCES measurements(id),
                time_of_meas BIGINT,
                distance DOUBLE PRECISION,
                angleHz DOUBLE PRECISION,
                angleVt DOUBLE PRECISION,
                q1 DOUBLE PRECISION,
                q2 DOUBLE PRECISION,
                q3 DOUBLE PRECISION,
                q4 DOUBLE PRECISION,
                q5 DOUBLE PRECISION,
                q6 DOUBLE PRECISION,
                q7 DOUBLE PRECISION
            )""")
            # Fail fast for schema mismatches instead of starting robot movement.
            cur.execute("""SELECT time_of_meas, distance, angleHz, angleVt,
                                  q1, q2, q3, q4, q5, q6, q7
                           FROM measurement_data LIMIT 0""")
        conn.commit()
    except Exception:
        conn.rollback()
        raise
