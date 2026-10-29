"""Local coordinate frame from three non-collinear Leica Cartesian points.

Origin P1, X toward P2, Y in the P1/P2/P3 half-plane, Z = X cross Y.
"""
import math


def table_frame_from_points(points):
    if len(points) != 3:
        raise ValueError("Exactly three table points are required")
    p1, p2, p3 = [[float(v) for v in p] for p in points]
    def subtract(a, b): return [x-y for x, y in zip(a, b)]
    def dot(a, b): return sum(x*y for x,y in zip(a,b))
    def unit(a):
        n = math.sqrt(dot(a,a))
        if n < 1e-9: raise ValueError("Coincident or collinear calibration points")
        return [v/n for v in a]
    x = unit(subtract(p2,p1))
    toward3 = subtract(p3,p1)
    y = unit([a-dot(toward3,x)*b for a,b in zip(toward3,x)])
    z = [x[1]*y[2]-x[2]*y[1], x[2]*y[0]-x[0]*y[2], x[0]*y[1]-x[1]*y[0]]
    return p1, x, y, z
