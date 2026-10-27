"""Validate and load joint targets [q1..q7], degrees, from CSV."""
import csv
import math


def load_positions(file_path):
    with open(file_path, encoding='utf-8-sig', newline='') as handle:
        sample = handle.read(4096)
        handle.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=',;\t')
            reader = csv.DictReader(handle, dialect=dialect)
        except csv.Error:
            reader = csv.DictReader(handle)
        required = ['point_id'] + [f'q{i}' for i in range(1, 8)]
        if not reader.fieldnames or any(name not in reader.fieldnames for name in required):
            raise ValueError('CSV must contain ' + ','.join(required))
        positions, ids = [], []
        for lineno, row in enumerate(reader, 2):
            point_id = str(row['point_id']).strip()
            if not point_id or point_id in ids:
                raise ValueError(f'Invalid or duplicate point_id on line {lineno}')
            angles = [float(row[name]) for name in required[1:]]
            if not all(math.isfinite(angle) for angle in angles):
                raise ValueError(f'Nonfinite joint angle on line {lineno}')
            ids.append(point_id)
            positions.append(angles)
        if not positions:
            raise ValueError('No target rows')
        return positions, ids