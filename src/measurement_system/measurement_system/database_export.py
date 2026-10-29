"""Export raw PostgreSQL measurements to CSV or HDF5 (no data transformations)."""
import csv
import os
from datetime import date, datetime
from pathlib import Path

import psycopg2
from psycopg2 import sql

ALLOWED_TABLES = ("measurement_data", "table_calibration_points", "robot_base_calibration_points", "robot_base_calibration_runs", "measurements", "robots")


def export_table(file_path, table="measurement_data"):
    """Export a single table. Return number of exported rows.

    Configure connection via PGHOST, PGPORT, PGDATABASE, PGUSER,
    PGPASSWORD (or ~/.pgpass). Never store DB credentials in source code.
    """
    if table not in ALLOWED_TABLES:
        raise ValueError(f"Unsupported table: {table}")
    file_path = Path(file_path)
    suffix = file_path.suffix.lower()
    if suffix not in (".csv", ".h5", ".hdf5"):
        raise ValueError("Use .csv, .h5 or .hdf5")

    from measurement_system.drivers.db_setup.connection_settings import connect_with_settings
    conn = connect_with_settings()
    try:
        with conn.cursor() as cur:
            cur.execute(sql.SQL("SELECT * FROM {}").format(sql.Identifier(table)))
            columns = [col[0] for col in cur.description]
            rows = cur.fetchall()
    finally:
        conn.close()

    if suffix == ".csv":
        with file_path.open("w", encoding="utf-8-sig", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(columns)
            writer.writerows(rows)
    else:
        import h5py
        import numpy as np

        with h5py.File(file_path, "w") as h5:
            group = h5.create_group("measurements")
            group.attrs["source_table"] = table
            group.attrs["row_count"] = len(rows)
            for index, column in enumerate(columns):
                values = [row[index] for row in rows]
                non_null = [v for v in values if v is not None]
                if non_null and all(isinstance(v, (int, float, bool)) for v in non_null):
                    # Float64 with NaN for SQL NULL; preserves typical measurement precision
                    arr = np.array([float(v) if v is not None else np.nan for v in values], dtype=np.float64)
                    group.create_dataset(column, data=arr, compression="gzip" if len(arr) else None)
                else:
                    # Datetime/text/empty columns are stored as UTF-8 strings.
                    dt = h5py.string_dtype(encoding="utf-8")
                    text_values = [v.isoformat() if isinstance(v, (date, datetime)) else ("" if v is None else str(v)) for v in values]
                    group.create_dataset(column, data=np.asarray(text_values, dtype=object), dtype=dt)
    return len(rows)