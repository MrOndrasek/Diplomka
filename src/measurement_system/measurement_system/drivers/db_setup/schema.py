"""Single authoritative PostgreSQL schema and non-destructive migrations.

All CREATE TABLE and ALTER TABLE statements live in this file only.
Existing records are retained; q1..q7 are renamed to q1_indicated..q7_indicated.
"""


def ensure_measurement_tables(conn):
    try:
        with conn.cursor() as cur:
            cur.execute("""CREATE TABLE IF NOT EXISTS robots (
                id SERIAL PRIMARY KEY, robot_name TEXT NOT NULL,
                robot_type TEXT, host TEXT, port INTEGER,
                is_active BOOLEAN NOT NULL DEFAULT FALSE
            )""")
            for name, kind in (("robot_type", "TEXT"), ("host", "TEXT"),
                               ("port", "INTEGER"),
                               ("is_active", "BOOLEAN NOT NULL DEFAULT FALSE")):
                cur.execute(f"ALTER TABLE robots ADD COLUMN IF NOT EXISTS {name} {kind}")
            cur.execute("""CREATE TABLE IF NOT EXISTS measurements (
                id SERIAL PRIMARY KEY,
                robot_id INTEGER REFERENCES robots(id),
                time_of_measuring TIMESTAMP DEFAULT NOW(), name TEXT NOT NULL
            )""")
            cur.execute("""CREATE TABLE IF NOT EXISTS measurement_data (
                id SERIAL PRIMARY KEY,
                measurement_id INTEGER REFERENCES measurements(id),
                time_of_meas BIGINT, distance DOUBLE PRECISION,
                angleHz DOUBLE PRECISION, angleVt DOUBLE PRECISION
            )""")
            # Rename historical measured joints without touching existing values.
            # IF NOT EXISTS for NEW columns, so migration is safe to repeat.
            for idx in range(1, 8):
                old = f"q{idx}"
                indicated = f"q{idx}_indicated"
                cur.execute("""SELECT EXISTS(
                    SELECT 1 FROM information_schema.columns
                    WHERE table_schema=current_schema()
                      AND table_name='measurement_data' AND column_name=%s
                )""", (old,))
                has_old = cur.fetchone()[0]
                if has_old:
                    cur.execute("""SELECT EXISTS(
                        SELECT 1 FROM information_schema.columns
                        WHERE table_schema=current_schema()
                          AND table_name='measurement_data' AND column_name=%s
                    )""", (indicated,))
                    if cur.fetchone()[0]:
                        raise RuntimeError(f"Both {old} and {indicated} exist: manual migration required")
                    cur.execute(f"ALTER TABLE measurement_data RENAME COLUMN {old} TO {indicated}")
                cur.execute(f"ALTER TABLE measurement_data ADD COLUMN IF NOT EXISTS {indicated} DOUBLE PRECISION")
                cur.execute(f"ALTER TABLE measurement_data ADD COLUMN IF NOT EXISTS q{idx}_wanted DOUBLE PRECISION")
            cur.execute("""CREATE TABLE IF NOT EXISTS table_calibration_points (
                id BIGSERIAL PRIMARY KEY,
                measurement_id INTEGER NOT NULL REFERENCES measurements(id),
                point_number INTEGER NOT NULL CHECK(point_number BETWEEN 1 AND 3),
                time_of_meas BIGINT NOT NULL, distance DOUBLE PRECISION NOT NULL,
                angleHz DOUBLE PRECISION NOT NULL, angleVt DOUBLE PRECISION NOT NULL,
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                UNIQUE(measurement_id, point_number)
            )""")
            # Preserve older calibration-B data, but no longer start that workflow.
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
                q1 DOUBLE PRECISION, q2 DOUBLE PRECISION, q3 DOUBLE PRECISION,
                q4 DOUBLE PRECISION, q5 DOUBLE PRECISION, q6 DOUBLE PRECISION,
                q7 DOUBLE PRECISION,
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                PRIMARY KEY(calibration_id, point_id)
            )""")
            # New 3-correspondence calibration uses existing run/point tables.
            cur.execute("""ALTER TABLE robot_base_calibration_runs
                ADD COLUMN IF NOT EXISTS transform_leica_from_robot JSONB""")
            cur.execute("""ALTER TABLE robot_base_calibration_runs
                ADD COLUMN IF NOT EXISTS rms_error_mm DOUBLE PRECISION""")
            cur.execute("""ALTER TABLE robot_base_calibration_runs
                ADD COLUMN IF NOT EXISTS leica_units JSONB""")
            cur.execute("""CREATE INDEX IF NOT EXISTS idx_base_calibration_measurement
                           ON robot_base_calibration_runs(measurement_id)""")
            cur.execute("""SELECT measurement_id, time_of_meas, distance, angleHz, angleVt,
                           q1_indicated,q2_indicated,q3_indicated,q4_indicated,
                           q5_indicated,q6_indicated,q7_indicated,
                           q1_wanted,q2_wanted,q3_wanted,q4_wanted,
                           q5_wanted,q6_wanted,q7_wanted FROM measurement_data LIMIT 0""")
        conn.commit()
    except Exception:
        conn.rollback()
        raise
