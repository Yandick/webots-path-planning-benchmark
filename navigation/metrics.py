"""Shared arc-length sampling and explicitly defined smoothness measurements."""
import numpy as np


def resample_path(path, spacing):
    path = np.asarray(path, dtype=float)
    lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    keep = np.r_[True, np.diff(lengths) > 1e-9]
    lengths, path = lengths[keep], path[keep]
    stations = np.r_[np.arange(0., lengths[-1], spacing), lengths[-1]]
    return np.column_stack([np.interp(stations, lengths, path[:, k]) for k in range(2)])


def path_metrics(path):
    """Menger curvature at a common 0.10m arc-length spacing, in 1/m."""
    p = resample_path(path, .10)
    a, b = p[1:-1]-p[:-2], p[2:]-p[1:-1]
    denominator = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)*np.linalg.norm(a+b, axis=1)
    curvature = 2*np.abs(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0])/np.maximum(denominator, 1e-12)
    return {"planned_peak_curvature": float(curvature.max()) if len(curvature) else 0.}


def control_metrics(rows, dt):
    """Command angular acceleration RMS while v > 0.02m/s, in rad/s^2.

    Excluding stopped samples prevents a long wait diluting the score.
    """
    omega = np.asarray([p['omega'] for p in rows])
    moving = np.asarray([abs(p['v']) > .02 for p in rows])
    accel = np.diff(np.r_[0., omega])/dt
    return {"moving_angular_accel_rms": float(np.sqrt(np.mean(accel[moving]**2))) if moving.any() else None}
