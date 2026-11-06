"""Export PostgreSQL to a complete hierarchical HDF5 or a single-table CSV.

No FK, model or robot transformations are performed. Identifiers, timestamps,
raw readings and joint vectors are retained for offline processing.
"""
from __future__ import annotations

import csv
import math
from datetime import date, datetime
from pathlib import Path
from uuid import UUID

from measurement_system.drivers.db_setup.connection_settings import connect_with_settings

TABLES = (
    "robots", "measurements", "measurement_data", "table_calibration_points",
    "robot_base_calibration_runs", "robot_base_calibration_points",
)
CSV_TABLES = TABLES


def _fetch(conn, table):
    """Read a known table, or return None if it is absent (old DB)."""
    from psycopg2 import sql
    with conn.cursor() as cur:
        cur.execute("SELECT to_regclass(%s)", (table,))
        if cur.fetchone()[0] is None:
            return None
        cur.execute(sql.SQL("SELECT * FROM {} ORDER BY 1").format(sql.Identifier(table)))
        cols = [desc[0] for desc in cur.description]
        return [dict(zip(cols, row)) for row in cur.fetchall()]


def _s(value):
    if value is None:
        return ""
    if isinstance(value, (datetime, date)):
        return value.isoformat()
    return str(value)


def _dataset(group, key, values):
    """Write a 1D dataset without discarding NULL information."""
    import h5py
    import numpy as np
    vals = list(values)
    nonnull = [v for v in vals if v is not None]
    if nonnull and all(isinstance(v, (bool, int, float)) for v in nonnull):
        if all(isinstance(v, (bool, int)) for v in nonnull) and len(nonnull) == len(vals):
            a = np.asarray(vals, dtype=np.int64)
        else:
            a = np.asarray([float(v) if v is not None else np.nan for v in vals], dtype=np.float64)
        group.create_dataset(key, data=a, compression="gzip" if len(a) else None)
    else:
        dtype = h5py.string_dtype("utf-8")
        group.create_dataset(key, data=np.asarray([_s(v) for v in vals], dtype=object), dtype=dtype)
        # Empty string is ambiguous, so expose a validity mask when NULLs exist.
        if any(v is None for v in vals):
            group.create_dataset(key + "_valid", data=np.asarray([v is not None for v in vals], dtype=np.bool_))


def _columns(group, rows, *, exclude=()):
    """Preserve ALL database columns, including future columns not known to exporter."""
    if not rows:
        return
    for col in rows[0].keys():
        if col not in exclude:
            _dataset(group, col.lower(), (row.get(col) for row in rows))


def _joints(group, rows, stem, column_patterns):
    """Convenience dense N×7 view; raw columns are retained independently."""
    import numpy as np
    for pattern in column_patterns:
        cols = [pattern.format(i=i) for i in range(1, 8)]
        if rows and all(col in rows[0] for col in cols):
            arr = np.asarray([
                [float(row[col]) if row[col] is not None else np.nan for col in cols]
                for row in rows
            ], dtype=np.float64)
            group.create_dataset(stem, data=arr, compression="gzip" if len(arr) else None)
            group[stem].attrs["columns"] = ",".join(cols)
            group[stem].attrs["units"] = "degrees"
            return
    group.create_dataset(stem, shape=(0, 7), dtype=np.float64)
    group[stem].attrs["unavailable"] = True


def _group_rows(parent, name, rows, joint_patterns=()):
    grp = parent.create_group(name)
    grp.attrs["row_count"] = len(rows)
    raw = grp.create_group("raw_columns")
    _columns(raw, rows)
    if rows:
        lower_names = {k.lower(): k for k in rows[0]}
        for canonical, options in (
            ("distance", ("distance",)),
            ("angle_hz", ("anglehz", "angle_hz")),
            ("angle_vt", ("anglevt", "angle_vt")),
            ("timestamp", ("time_of_meas", "measured_at")),
            ("record_id", ("id", "point_id")),
        ):
            match = next((lower_names[o] for o in options if o in lower_names), None)
            if match:
                _dataset(grp, canonical, (row[match] for row in rows))
    for stem, patterns in joint_patterns:
        _joints(grp, rows, stem, patterns)
    return grp


def _write_hdf5(file_path, data):
    import h5py
    total = sum(len(data.get(name) or []) for name in TABLES)
    with h5py.File(file_path, "w") as h5:
        h5.attrs["format_version"] = "2.0"
        h5.attrs["export_format"] = "diplomka_hierarchical"
        h5.attrs["total_source_rows"] = total
        h5.attrs["note"] = "Raw PostgreSQL data; no calibration or coordinate conversions"
        robots = h5.create_group("robots")
        measurements = h5.create_group("measurements")
        calibrations = h5.create_group("calibrations")
        table_group = calibrations.create_group("table")
        base_group = calibrations.create_group("robot_base")

        for robot in data.get("robots") or []:
            g = robots.create_group(f"robot_{robot['id']}")
            for k, v in robot.items():
                if v is not None:
                    g.attrs[k] = _s(v) if not isinstance(v, (int, float, bool)) else v
            # Passwords / connection credentials are not present in this table.

        measured = data.get("measurement_data") or []
        series = data.get("measurements") or []
        valid_ids = set()
        for series_row in series:
            mid = series_row["id"]
            valid_ids.add(mid)
            points = [r for r in measured if r.get("measurement_id") == mid]
            g = _group_rows(measurements, f"measurement_{mid}", points, joint_patterns=(
                ("q_wanted", ("q{i}_wanted",)),
                ("q_indicated", ("q{i}_indicated", "q{i}")),
            ))
            for k, v in series_row.items():
                if v is not None:
                    g.attrs[k] = _s(v) if not isinstance(v, (int, float, bool)) else v
        # Do not silently lose old rows missing a measurement_id.
        orphan = [r for r in measured if r.get("measurement_id") not in valid_ids]
        if orphan:
            _group_rows(measurements, "unassigned", orphan, joint_patterns=(
                ("q_wanted", ("q{i}_wanted",)),
                ("q_indicated", ("q{i}_indicated", "q{i}")),
            ))

        table_rows = data.get("table_calibration_points") or []
        mids = sorted({r.get("measurement_id") for r in table_rows}, key=lambda x: (x is None, str(x)))
        for mid in mids:
            rows = [r for r in table_rows if r.get("measurement_id") == mid]
            g = table_group.create_group(f"measurement_{mid}" if mid is not None else "unassigned")
            _group_rows(g, "points", rows)
            g.attrs["source_table"] = "table_calibration_points"

        runs = data.get("robot_base_calibration_runs") or []
        base_rows = data.get("robot_base_calibration_points") or []
        uuids = {str(r["calibration_id"]) for r in runs} | {str(r["calibration_id"]) for r in base_rows}
        for uid in sorted(uuids):
            rows = [r for r in base_rows if str(r["calibration_id"]) == uid]
            g = _group_rows(base_group, f"calibration_{uid}", rows, joint_patterns=(
                ("joints", ("q{i}", "q{i}_indicated")),
            ))
            g.attrs["calibration_id"] = uid
            matching = next((r for r in runs if str(r["calibration_id"]) == uid), None)
            if matching:
                for key, value in matching.items():
                    if value is not None:
                        g.attrs[key] = value if isinstance(value, (bool, int, float)) else _s(value)
        # Persist empty table definitions for completeness.
        for table, rows in data.items():
            if rows is None:
                h5.attrs[f"missing_table_{table}"] = True
    return total


def export_table(file_path, table="measurement_data"):
    """CSV: export selected SQL table. HDF5: export all robot/measurement/calibration data."""
    file_path = Path(file_path)
    suffix = file_path.suffix.lower()
    if suffix not in (".csv", ".h5", ".hdf5"):
        raise ValueError("Use .csv, .h5 or .hdf5")
    if suffix == ".csv" and table not in CSV_TABLES:
        raise ValueError(f"Unsupported CSV table: {table}")
    conn = connect_with_settings()
    try:
        if suffix == ".csv":
            records = _fetch(conn, table)
            if records is None:
                raise RuntimeError(f"Table {table} not found")
            if not records:
                # Preserve column names even for an empty table.
                with conn.cursor() as cur:
                    from psycopg2 import sql
                    cur.execute(sql.SQL("SELECT * FROM {} LIMIT 0").format(sql.Identifier(table)))
                    columns = [d[0] for d in cur.description]
            else:
                columns = list(records[0])
            with file_path.open("w", encoding="utf-8-sig", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(columns)
                writer.writerows([[r.get(k) for k in columns] for r in records])
            return len(records)
        data = {table_name: _fetch(conn, table_name) for table_name in TABLES}
    finally:
        conn.close()
    return _write_hdf5(file_path, data)
