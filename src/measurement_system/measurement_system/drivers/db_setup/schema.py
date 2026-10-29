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
            cur.execute("""CREATE TABLE IF NOT EXISTS table_calibration_points (
                id BIGSERIAL PRIMARY KEY,
                measurement_id INTEGER NOT NULL REFERENCES measurements(id),
                point_number INTEGER NOT NULL CHECK(point_number BETWEEN 1 AND 3),
                time_of_meas BIGINT NOT NULL,
                distance DOUBLE PRECISION NOT NULL,
                angleHz DOUBLE PRECISION NOT NULL,
                angleVt DOUBLE PRECISION NOT NULL,
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                UNIQUE(measurement_id, point_number)
            )""")
            cur.execute("""CREATE TABLE IF NOT EXISTS robot_base_calibration_runs (
                calibration_id UUID PRIMARY KEY,
                measurement_id INTEGER REFERENCES measurements(id),
                started_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                status TEXT NOT NULL DEFAULT 'RUNNING'
            )""")
            cur.execute("""CREATE TABLE IF NOT EXISTS robot_base_calibration_points (
                calibration_id UUID NOT NULL,
                point_id TEXT NOT NULL,
                measured_at BIGINT NOT NULL,
                distance DOUBLE PRECISION NOT NULL,
                angle_hz DOUBLE PRECISION NOT NULL,
                angle_vt DOUBLE PRECISION NOT NULL,
                q1 DOUBLE PRECISION NOT NULL, q2 DOUBLE PRECISION NOT NULL,
                q3 DOUBLE PRECISION NOT NULL, q4 DOUBLE PRECISION NOT NULL,
                q5 DOUBLE PRECISION NOT NULL, q6 DOUBLE PRECISION NOT NULL,
                q7 DOUBLE PRECISION NOT NULL,
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                PRIMARY KEY (calibration_id, point_id)
            )""")
            cur.execute("""SELECT calibration_id, point_id, measured_at, distance,
                              angle_hz, angle_vt, q1,q2,q3,q4,q5,q6,q7
                           FROM robot_base_calibration_points LIMIT 0""")
            # Detect incompatible pre-existing table schemas early.
            cur.execute("""SELECT measurement_id, point_number, time_of_meas,
                              distance, angleHz, angleVt FROM table_calibration_points LIMIT 0""")
            # Fail fast for schema mismatches instead of starting robot movement.
            cur.execute("""SELECT measurement_id, time_of_meas, distance, angleHz, angleVt,
                                  q1, q2, q3, q4, q5, q6, q7
                           FROM measurement_data LIMIT 0""")
        conn.commit()
    except Exception:
        conn.rollback()
        raise
