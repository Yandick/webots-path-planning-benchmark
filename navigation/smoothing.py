"""Dev1ce's method: Voronoi reference with safe-bubble QP smoothing.

Reproduces Wen et al. (2024)'s smoothing objective and bounds of G2VD, Section IV-D,
Equations (5)-(8), DOI 10.1109/TASE.2024.3398996. The input here is the
shared geometric Voronoi route, rather than the paper's state-lattice search.
"""
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize

from .classical import voronoi_route
from .metrics import resample_path


def safe_bubbles(grid, reference):
    indices = np.array([grid.world_to_grid(*p) for p in reference])
    # Convert the raster EDT to a conservative continuous-position bound.
    distances = grid.dist_map[indices[:, 1], indices[:, 0]]-grid.resolution/np.sqrt(2)
    bubble = np.maximum(0., distances/np.sqrt(2)-grid.inflation_radius)
    lower, upper = reference-bubble[:, None], reference+bubble[:, None]
    lower[[0, -1]] = upper[[0, -1]] = reference[[0, -1]]
    return lower, upper


def smooth_reference(grid, reference):
    """Convex quadratic objective with fixed endpoints and safe square bounds."""
    reference = resample_path(np.asarray(reference), .10)
    smoothness_weight = 1000.
    n = len(reference)
    if n < 3:
        return reference
    lower, upper = safe_bubbles(grid, reference)

    def objective(flat):
        p = flat.reshape(n, 2)
        second = p[2:]-2*p[1:-1]+p[:-2]
        deviation = p-reference
        value = smoothness_weight*np.sum(second**2)+np.sum(deviation**2)
        gradient = 2.*deviation
        gradient[:-2] += 2*smoothness_weight*second
        gradient[1:-1] -= 4*smoothness_weight*second
        gradient[2:] += 2*smoothness_weight*second
        return value, gradient.ravel()

    result = minimize(objective, reference.ravel(), jac=True, method="L-BFGS-B",
                      bounds=list(zip(lower.ravel(), upper.ravel())),
                      options=dict(ftol=1e-13, gtol=1e-8, maxiter=2000))
    if not result.success:
        raise RuntimeError(f"Safe-bubble QP did not converge: {result.message}")
    optimized = result.x.reshape(n, 2)
    # Cubic interpolation follows Section IV-E; every interpolated segment is
    # checked against the same inflated grid used by all global planners.
    stations = np.r_[0., np.cumsum(np.linalg.norm(np.diff(optimized, axis=0), axis=1))]
    curve = CubicSpline(stations, optimized, bc_type="natural")
    candidate = curve(np.r_[np.arange(0., stations[-1], .04), stations[-1]])
    for path in (candidate, optimized):
        if all(grid.is_line_collision_free(a, b) for a, b in zip(path[:-1], path[1:])):
            return path
    return None


def voronoi_qp_path(grid, start, goal):
    """Plan with Dev1ce's method; retain the stable callable name for compatibility."""
    reference = voronoi_route(grid, start, goal)
    return None if reference is None else smooth_reference(grid, reference)
