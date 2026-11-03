#!/usr/bin/env python3
import psycopg2


def main():
    if conn:
        print("Connection to DB succesfull")
    else:
        print("Connection to DB failed")
    create_table()


def create_table():
    """Backward compatible entry-point; DDL is centralized in schema.py."""
    from measurement_system.drivers.db_setup.schema import ensure_measurement_tables
    connection = globals().get("conn")
    if connection is None or connection is False:
        raise RuntimeError("Database connection unavailable")
    ensure_measurement_tables(connection)


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
