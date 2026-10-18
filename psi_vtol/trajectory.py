"""Speed changes and paths that the vehicle can follow without a step in its attitude.

A step in the commanded speed is a step in PX4's tilt setpoint: the nose jumps and, through
pusher assist, the pusher is cut at once. Everything here changes speed with a limited jerk:
the acceleration is built up over ramp_s, held at its limit, and taken off again over ramp_s.

Nothing here knows about PX4, MAVLink or ROS; distances are metres along a path.
"""
import bisect
import math


class SpeedRamp:
    """Speed from v0 to v1, at most accel, with the acceleration built up and taken off over ramp_s.

    With enough speed to change, the acceleration is a trapezoid and the change takes
    |v1 - v0| / accel + ramp_s; with little, a triangle that never reaches accel.
    """

    def __init__(self, v0, v1, accel, ramp_s):
        if accel <= 0.0 or ramp_s < 0.0:
            raise ValueError("accel must be positive and ramp_s not negative")
        self.v0, self.v1 = float(v0), float(v1)
        change = abs(self.v1 - self.v0)
        sign = 1.0 if self.v1 >= self.v0 else -1.0
        if ramp_s == 0.0:                                   # a plain ramp at the limit
            pieces = [(change / accel, 0.0, sign * accel)]
        elif change >= accel * ramp_s:
            jerk = accel / ramp_s
            pieces = [(ramp_s, sign * jerk, None), (change / accel - ramp_s, 0.0, None),
                      (ramp_s, -sign * jerk, None)]
        else:                                               # never reaches the limit
            jerk = accel / ramp_s
            half = math.sqrt(change / jerk)
            pieces = [(half, sign * jerk, None), (half, -sign * jerk, None)]
        # (start time, duration, jerk, acceleration, speed, distance) at the start of each piece
        self._pieces, t, a, v, s = [], 0.0, 0.0, self.v0, 0.0
        for duration, jerk, held in pieces:
            if held is not None:                            # ramp_s == 0: constant acceleration
                a = held
            self._pieces.append((t, duration, jerk, a, v, s))
            t, s = t + duration, s + v * duration + a * duration ** 2 / 2 + jerk * duration ** 3 / 6
            v, a = v + a * duration + jerk * duration ** 2 / 2, a + jerk * duration
        self.duration, self.length = t, s

    def state(self, t):
        """(distance, speed, acceleration) at time t after the start; held after the end."""
        if t >= self.duration:
            return self.length + self.v1 * (t - self.duration), self.v1, 0.0
        t = max(0.0, t)
        for start, duration, jerk, a, v, s in reversed(self._pieces):
            if t >= start:
                tau = t - start
                return (s + v * tau + a * tau ** 2 / 2 + jerk * tau ** 3 / 6,
                        v + a * tau + jerk * tau ** 2 / 2, a + jerk * tau)
        raise AssertionError("unreachable")


def stopping_distance(speed, accel, ramp_s):
    """How far the vehicle goes while a SpeedRamp takes it from speed to a stop."""
    return SpeedRamp(speed, 0.0, accel, ramp_s).length


def _reachable(v0, length, limit, accel, ramp_s):
    """The highest speed up to limit that a SpeedRamp from v0 reaches within length."""
    if SpeedRamp(v0, limit, accel, ramp_s).length <= length:
        return limit
    low, high = v0, limit
    for _ in range(50):
        mid = (low + high) / 2
        low, high = (mid, high) if SpeedRamp(v0, mid, accel, ramp_s).length <= length else (low, mid)
    return low


def _turn_shape(turn, radius, transition_m, ds=0.02):
    """A turn by `turn` radians that eases its curvature in and out over transition_m.

    Returned in a frame where the turn starts at the origin heading along +x, as arrays of
    (distance, x, y, heading, curvature), with the curvature it reaches. A positive turn is to
    the right: heading is measured from north towards east, so +y is to the right.
    """
    sign, total = math.copysign(1.0, turn), abs(turn)
    peak = 1.0 / radius
    if total / peak >= transition_m:
        steady = total / peak - transition_m
    else:                               # a small turn never reaches the full curvature
        peak, steady = total / transition_m, 0.0
    length = 2 * transition_m + steady
    n = max(10, int(math.ceil(length / ds)))
    s = [length * i / n for i in range(n + 1)]
    def curvature(d):
        if d < transition_m:
            return peak * d / transition_m
        if d > transition_m + steady:
            return peak * max(0.0, length - d) / transition_m
        return peak
    xs, ys, hs, ks = [0.0], [0.0], [0.0], [0.0]
    for a, b in zip(s, s[1:]):          # midpoint rule for the heading, then the position
        k_mid = curvature((a + b) / 2)
        h_mid = hs[-1] + sign * k_mid * (b - a) / 2
        hs.append(hs[-1] + sign * k_mid * (b - a))
        xs.append(xs[-1] + math.cos(h_mid) * (b - a))
        ys.append(ys[-1] + math.sin(h_mid) * (b - a))
        ks.append(sign * curvature(b))
    hs[-1] = turn                       # exact, so the next straight line starts aligned
    return s, xs, ys, hs, ks, peak


class Path:
    """Straight lines through waypoints (north, east), joined by eased turns at the inner ones.

    Each inner waypoint is passed at a distance: the turn starts and ends where it leaves the
    two lines, the same distance before and after the waypoint. A turn that does not fit
    between its neighbours is made tighter, down to min_radius_m.
    """

    def __init__(self, points, radius_m, transition_m, min_radius_m=None, ds=0.05):
        pts = [tuple(map(float, p[:2])) for p in points]
        if len(pts) < 2:
            raise ValueError("a path needs at least two points")
        legs = []
        for a, b in zip(pts, pts[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            if length < 1e-6:
                raise ValueError("two consecutive points of a path coincide")
            legs.append((a, b, length, math.atan2(b[1] - a[1], b[0] - a[0])))
        turns = [math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))
                 for (_, _, _, h1), (_, _, _, h2) in zip(legs, legs[1:])]
        min_radius_m = radius_m if min_radius_m is None else min_radius_m
        radii = [radius_m] * len(turns)
        while True:                     # make turns tighter until their tangents fit the legs
            shapes = [_turn_shape(t, r, transition_m) if abs(t) > 1e-6 else None
                      for t, r in zip(turns, radii)]
            tangents = [0.0 if sh is None else self._tangent(sh, t) for sh, t in zip(shapes, turns)]
            ends = [0.0] + tangents + [0.0]
            short = [i for i, (_, _, length, _) in enumerate(legs) if ends[i] + ends[i + 1] > length]
            if not short:
                break
            shrink = {k for i in short for k in (i - 1, i) if 0 <= k < len(turns)}
            if any(radii[k] * 0.9 < min_radius_m for k in shrink):
                raise ValueError(f"leg {short[0] + 1} is too short for its turns at a radius of "
                                 f"{min_radius_m:g} m")
            for k in shrink:
                radii[k] *= 0.9
        self.radii = radii
        # sections: ("line", leg index, length) and ("turn", corner index, length, peak curvature)
        self.sections, s_all, n_all, e_all, h_all, k_all = [], [], [], [], [], []

        def add(points_s, ns, es, hs, ks):
            offset = s_all[-1] if s_all else 0.0
            start = 1 if s_all else 0
            s_all.extend(offset + d for d in points_s[start:])
            n_all.extend(ns[start:]); e_all.extend(es[start:])
            h_all.extend(hs[start:]); k_all.extend(ks[start:])

        position = pts[0]
        for i, (a, b, length, heading) in enumerate(legs):
            straight = length - ends[i] - ends[i + 1]
            count = max(1, int(math.ceil(straight / ds)))
            ds_line = [straight * j / count for j in range(count + 1)]
            add(ds_line, [position[0] + math.cos(heading) * d for d in ds_line],
                [position[1] + math.sin(heading) * d for d in ds_line], [heading] * (count + 1),
                [0.0] * (count + 1))
            self.sections.append(("line", i, straight))
            position = (position[0] + math.cos(heading) * straight,
                        position[1] + math.sin(heading) * straight)
            if i < len(turns) and shapes[i] is not None:
                d, xs, ys, hs, ks, peak = shapes[i]
                c, sn = math.cos(heading), math.sin(heading)
                add(d, [position[0] + x * c - y * sn for x, y in zip(xs, ys)],
                    [position[1] + x * sn + y * c for x, y in zip(xs, ys)],
                    [heading + h for h in hs], ks)
                self.sections.append(("turn", i, d[-1], peak))
                position = (n_all[-1], e_all[-1])
        for i in range(1, len(h_all)):    # continuous, so that interpolating it never jumps
            h_all[i] = h_all[i - 1] + math.atan2(math.sin(h_all[i] - h_all[i - 1]),
                                                 math.cos(h_all[i] - h_all[i - 1]))
        self.length = s_all[-1]
        self._s, self._n, self._e = s_all, n_all, e_all
        self._h, self._k = h_all, k_all
        self.legs = legs

    @staticmethod
    def _tangent(shape, turn):
        _, xs, ys, _, _, _ = shape
        return xs[-1] - ys[-1] * math.cos(turn) / math.sin(turn) if abs(math.sin(turn)) > 1e-9 \
            else math.inf

    def at(self, s):
        """(north, east, heading, curvature) at distance s along the path. The heading is
        continuous along the path, so it can leave -pi..pi."""
        s = min(max(s, 0.0), self.length)
        i = max(0, min(len(self._s) - 2, bisect.bisect_right(self._s, s) - 1))
        s0, s1 = self._s[i], self._s[i + 1]
        w = 0.0 if s1 == s0 else (s - s0) / (s1 - s0)
        lerp = lambda v: v[i] + (v[i + 1] - v[i]) * w
        return lerp(self._n), lerp(self._e), lerp(self._h), lerp(self._k)

    def polyline(self, step_m=0.5):
        """Points along the path every step_m: what the vehicle was meant to fly over."""
        count = max(1, int(math.ceil(self.length / step_m)))
        return [self.at(self.length * j / count)[:2] for j in range(count + 1)]


class SpeedPlan:
    """Speed along a Path: from rest to rest, at most cruise_mps on lines, and slow enough in
    each turn that the sideways acceleration stays below lateral_mps2.

    Speed changes are SpeedRamps. state(t) gives the distance along the path, the speed, the
    acceleration along it, and the label of the piece flown: ("accel" | "cruise" | "decel" |
    "corner", leg or corner index).
    """

    def __init__(self, path, cruise_mps, accel_mps2, ramp_s, lateral_mps2):
        sections = path.sections
        limit = [cruise_mps if sec[0] == "line"
                 else min(cruise_mps, math.sqrt(lateral_mps2 / sec[3])) for sec in sections]
        # speed where each section starts and ends: rest at both ends, the turn's speed between
        edge = [0.0] + [min(limit[i], limit[i + 1]) for i in range(len(sections) - 1)] + [0.0]
        for _ in range(3):              # forward and backward: what the lines can reach
            for i, sec in enumerate(sections):
                if sec[0] == "line":
                    edge[i + 1] = min(edge[i + 1], _reachable(edge[i], sec[2], limit[i], accel_mps2, ramp_s))
                else:
                    edge[i + 1] = edge[i] = min(edge[i], edge[i + 1])
            for i in reversed(range(len(sections))):
                sec = sections[i]
                if sec[0] == "line":
                    edge[i] = min(edge[i], _reachable(edge[i + 1], sec[2], limit[i], accel_mps2, ramp_s))
                else:
                    edge[i + 1] = edge[i] = min(edge[i], edge[i + 1])
        # pieces: (label, index, start time, start distance, ramp or constant speed, duration, length)
        self.pieces, t, s = [], 0.0, 0.0
        for i, sec in enumerate(sections):
            v_in, v_out = edge[i], edge[i + 1]
            if sec[0] == "turn":
                if sec[2] > 0.0:
                    duration = sec[2] / max(v_in, 1e-6)
                    self.pieces.append(("corner", sec[1], t, s, v_in, duration, sec[2]))
                    t, s = t + duration, s + sec[2]
                continue
            low, high = max(v_in, v_out), limit[i]
            fits = lambda p: (SpeedRamp(v_in, p, accel_mps2, ramp_s).length
                              + SpeedRamp(p, v_out, accel_mps2, ramp_s).length) <= sec[2] + 1e-9
            if not fits(low):
                high = low
            elif not fits(high):
                for _ in range(50):
                    mid = (low + high) / 2
                    low, high = (mid, high) if fits(mid) else (low, mid)
            peak = low if not fits(high) else high
            up, down = SpeedRamp(v_in, peak, accel_mps2, ramp_s), SpeedRamp(peak, v_out, accel_mps2, ramp_s)
            steady = max(0.0, sec[2] - up.length - down.length)
            for label, ramp, length in (("accel", up, up.length), ("cruise", peak, steady),
                                        ("decel", down, down.length)):
                duration = ramp.duration if isinstance(ramp, SpeedRamp) else length / max(peak, 1e-6)
                if duration > 1e-9 and length > 1e-9:
                    self.pieces.append((label, sec[1], t, s, ramp, duration, length))
                    t, s = t + duration, s + length
        self.duration, self.length = t, s
        self.top_speed = max([p[4] if not isinstance(p[4], SpeedRamp) else max(p[4].v0, p[4].v1)
                              for p in self.pieces] or [0.0])

    def state(self, t):
        """(distance, speed, acceleration along the path, (label, index)) at time t."""
        if not self.pieces:
            return 0.0, 0.0, 0.0, ("decel", 0)
        if t >= self.duration:
            label, index = self.pieces[-1][:2]
            return self.length, 0.0, 0.0, (label, index)
        for label, index, start, s0, ramp, duration, length in self.pieces:
            if t < start + duration:
                tau = max(0.0, t - start)
                if isinstance(ramp, SpeedRamp):
                    d, v, a = ramp.state(tau)
                    return s0 + d, v, a, (label, index)
                return s0 + ramp * tau, ramp, 0.0, (label, index)
        raise AssertionError("unreachable")
