"""Estimate rigid transform Leica <- robot from 3 matched Cartesian points.

Units: mm. Leica azimuth and zenith angles assumed radians; validate
instrument-specific angle conventions before real calibration.
"""
import math
import numpy as np


def leica_cartesian(distance, angle_hz, angle_vt, *,
                    distance_unit="mm", angle_unit="rad",
                    vertical_convention="zenith"):
    d, hz, vt = map(float, (distance, angle_hz, angle_vt))
    if not all(map(math.isfinite, (d, hz, vt))) or d <= 0:
        raise ValueError("Invalid Leica measurement")
    distance_factor = {"mm": 1.0, "m": 1000.0}.get(distance_unit)
    angle_factor = {"rad": 1.0, "deg": math.pi/180.0,
                    "gon": math.pi/200.0}.get(angle_unit)
    if distance_factor is None or angle_factor is None:
        raise ValueError("Unknown Leica distance or angle unit")
    d *= distance_factor
    hz *= angle_factor
    vt *= angle_factor
    if vertical_convention == "elevation":
        vt = math.pi/2 - vt
    elif vertical_convention != "zenith":
        raise ValueError("Unknown Leica vertical-angle convention")
    # Convert vertical zenith angle measured from +Z.
    return np.array([d * math.sin(vt) * math.cos(hz),
                     d * math.sin(vt) * math.sin(hz),
                     d * math.cos(vt)], dtype=float)


def estimate_leica_from_robot(robot_points, leica_points):
    a = np.asarray(robot_points, dtype=float)
    b = np.asarray(leica_points, dtype=float)
    if a.shape != (3, 3) or b.shape != (3, 3) or not (np.isfinite(a).all() and np.isfinite(b).all()):
        raise ValueError("Need exactly three finite 3D correspondences")
    for points, label in ((a, "Robot"), (b, "Leica")):
        span = np.linalg.norm(points.max(axis=0) - points.min(axis=0))
        area = np.linalg.norm(np.cross(points[1]-points[0], points[2]-points[0]))
        if span < 20 or area < 1e-3 * span * span:
            raise ValueError(f"{label}: points too close or nearly collinear")
    ac = a.mean(axis=0)
    bc = b.mean(axis=0)
    u, s, vt = np.linalg.svd((a-ac).T @ (b-bc))
    flip = np.eye(3)
    flip[2, 2] = np.linalg.det(u @ vt)
    rotation = vt.T @ flip @ u.T
    translation = bc - rotation @ ac
    transformed = (rotation @ a.T).T + translation
    rmse = float(np.sqrt(np.mean(np.sum((transformed-b)**2, axis=1))))
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform, rmse
