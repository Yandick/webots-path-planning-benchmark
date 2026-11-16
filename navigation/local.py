"""Damped potential-field control and normalized Dynamic Window control.

APF: Khatib (1986), Sections 3-4, gradient forces and dissipative feedback.
DWA: Fox, Burgard & Thrun (1997), Section 4, reachable and admissible
velocities; normalized heading, forward collision distance and speed.
Both receive a reference-path target, as in hierarchical navigation.
"""
import math
import numpy as np


def apf_command(pose, velocity, target, points, dt, config):
    delta = np.asarray(target)-pose[:2]
    force = 2.*delta/max(1., np.linalg.norm(delta))
    if len(points):
        away = pose[:2]-points
        distances = np.linalg.norm(away, axis=1)
        i = int(np.argmin(distances))
        gap = max(float(distances[i])-config.radius, .03)
        if gap < .50:
            force += .008*(1/gap-1/.50)/gap**2*away[i]/max(float(distances[i]), 1e-9)
    actual = velocity[0]*np.array([math.cos(pose[2]), math.sin(pose[2])])
    desired = actual+dt*(force-2.*actual)
    magnitude = float(np.linalg.norm(desired))
    if magnitude < 1e-9:
        return 0., 0.
    error = (math.atan2(desired[1], desired[0])-pose[2]+math.pi)%(2*math.pi)-math.pi
    return min(config.max_speed, magnitude)*max(0., math.cos(error)), 2.*error


def collision_distance(pose, v, w, points, radius, maximum=5.):
    """Forward arc length to first contact with any inflated measured hit.

    Circle-circle and ray-circle intersections cover the continuous arc;
    collision checks do not depend on a rollout sampling interval.
    """
    result = np.full(len(v), maximum)
    if not len(points):
        return result
    delta = points-pose[:2]
    current = float(np.linalg.norm(delta, axis=1).min())
    stopped = v < 1e-8
    result[stopped] = max(0., current-radius)
    straight = (np.abs(w)<1e-7)&~stopped
    if straight.any():
        h = np.array([math.cos(pose[2]), math.sin(pose[2])])
        along = delta@h
        side2 = np.sum(delta**2, axis=1)-along**2
        root = np.sqrt(np.maximum(radius**2-side2, 0.))
        contacts = np.where((side2<=radius**2)&(along+root>=0), np.maximum(0., along-root), maximum)
        result[straight] = min(maximum, float(contacts.min()))
    curved = ~straight&~stopped
    if curved.any():
        turn = v[curved]/w[curved]
        center = pose[:2]+turn[:, None]*np.array([-math.sin(pose[2]), math.cos(pose[2])])
        r = np.abs(turn)
        offset = points[None, :, :]-center[:, None, :]
        distance = np.linalg.norm(offset, axis=2)
        cosine = (r[:, None]**2+distance**2-radius**2)/np.maximum(2*r[:, None]*distance, 1e-12)
        intersects = (np.abs(cosine)<=1.)
        phase = np.arctan2(offset[:, :, 1], offset[:, :, 0])
        half = np.arccos(np.clip(cosine, -1., 1.))
        start = np.arctan2(pose[1]-center[:, 1], pose[0]-center[:, 0])
        direction = np.sign(w[curved])[:, None]
        a = (direction*(phase-half-start[:, None]))%(2*np.pi)
        b = (direction*(phase+half-start[:, None]))%(2*np.pi)
        contact = r[:, None]*np.minimum(a, b)
        contact = np.where(intersects, contact, maximum)
        result[curved] = np.minimum(maximum, contact.min(axis=1))
    if current <= radius:
        result[~stopped] = 0.
    return result


def dwa_command(pose, velocity, target, points, dt, config):
    c = config
    vv, ww = np.meshgrid(np.linspace(max(0., velocity[0]-c.acceleration*dt),
                                     min(c.max_speed, velocity[0]+c.acceleration*dt), 7),
                         np.linspace(max(-c.max_omega, velocity[1]-c.angular_acceleration*dt),
                                     min(c.max_omega, velocity[1]+c.angular_acceleration*dt), 31))
    v, w = vv.ravel(), ww.ravel()
    free = collision_distance(pose, v, w, points, c.radius+c.margin)
    valid = free >= v*v/(2*c.acceleration)+v*dt
    valid &= np.maximum(np.abs(v-.15*w), np.abs(v+.15*w)) <= .5+1e-9
    if len(points):
        current = float(np.linalg.norm(points-pose[:2], axis=1).min())
        if c.radius+.03 < current < c.radius+c.margin:
            # The comfort margin must not prohibit a physically safe escape.
            # Use the same increasing-clearance rule as the shared brake layer.
            times = (dt+v/c.acceleration)[:, None]*np.linspace(0., 1., 12)[None, :]
            headings = pose[2]+w[:, None]*times/2
            positions = pose[:2]+v[:, None, None]*times[:, :, None]*np.stack((np.cos(headings), np.sin(headings)), axis=2)
            gap = np.linalg.norm(positions[:, :, None, :]-points[None, None, :, :], axis=3).min(axis=2)
            physical_free = collision_distance(pose, v, w, points, c.radius+.03)
            escape = (gap.min(axis=1)>=current-.002)&(gap[:, -1]>current+.001)
            escape &= physical_free >= v*v/(2*c.acceleration)+v*dt
            escape &= np.maximum(np.abs(v-.15*w), np.abs(v+.15*w)) <= .5+1e-9
            valid |= escape
        # Circular bodies can turn in place within the comfort margin.
        valid |= (v<1e-8)&(current>c.radius+.03)
    yaw = pose[2]+w*c.horizon
    straight = np.abs(w)<1e-7
    turn = v/np.where(straight, 1., w)
    end_x = pose[0]+np.where(straight, v*c.horizon*math.cos(pose[2]), turn*(np.sin(yaw)-math.sin(pose[2])))
    end_y = pose[1]+np.where(straight, v*c.horizon*math.sin(pose[2]), turn*(math.cos(pose[2])-np.cos(yaw)))
    desired = np.arctan2(target[1]-end_y, target[0]-end_x)
    heading = 1.-np.abs((desired-yaw+np.pi)%(2*np.pi)-np.pi)/np.pi
    # Saturate clearance at one meter: distant obstacles beyond the local
    # maneuver must not reward tiny closed circles over useful forward motion.
    clearance = np.clip(free/1., 0., 1.)
    speed = v/c.max_speed
    score = .60*heading+.25*clearance+.15*speed
    score[~valid] = -np.inf
    if not valid.any():
        return max(0., velocity[0]-c.acceleration*dt), 0.
    k = int(np.argmax(score))
    return float(v[k]), float(w[k])
