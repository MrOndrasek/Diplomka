#!/usr/bin/env python3
import psycopg2


def main():
    if conn:
        print("Connection to DB succesfull")
    else:
        print("Connection to DB failed")
    create_table()


def create_table():
    print("created")
    cur = conn.cursor()
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
    
    cur.close()
    conn.commit()


def get_connection():
    """Same connection method used by database, calibration and export."""
    from measurement_system.drivers.db_setup.connection_settings import connect_with_settings
    try:
        return connect_with_settings()
    except Exception as exc:
        print(f"PostgreSQL connection failed: {exc}")
        return False


if __name__ == "__main__":
    conn = get_connection()
    if conn:
        try:
            create_table()
        finally:
            conn.close()
