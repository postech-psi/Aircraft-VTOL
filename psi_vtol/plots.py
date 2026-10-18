#!/usr/bin/env python3
"""Figures of the simulation report. Each one answers one question.

    python3 -m psi_vtol.plots                          # the vehicle and the wing share only
    python3 -m psi_vtol.plots --flight out/square.json
    python3 -m psi_vtol.plots --oscillation out/calm.json out/wobbled.json 2
    python3 -m psi_vtol.plots --braking "Old" out/old/*.json --braking "Eased" out/new/*.json
    python3 -m psi_vtol.plots --wind a/wind.json "10 m/s, straight" --wind b/wind.json "5 m/s, square"
    python3 -m psi_vtol.plots --oat out/2026_year_end/aeroprop-v2/oat.json
    python3 -m psi_vtol.plots --speeds out/2026_year_end/aeroprop-v2/scenarios.json
    python3 -m psi_vtol.plots --silence out/point.json "Point" out/velocity.json "Velocity"

Figures go to missions/<mission>/figures/<model revision>/ as PNG and PDF. The pusher-thrust
figure is drawn from the cruise segments of the wind studies given with --wind.
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from psi_vtol import config  # noqa: E402
from psi_vtol.sim import gz_model, metrics  # noqa: E402

CFG = FIGURES = None            # the mission being drawn; set by main()

# Ink and surfaces
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
SURFACE, GRID, AXIS, BAND = "#fcfcfb", "#e1e0d9", "#c3c2b7", "#f0efec"
# Two categorical hues, one blue ramp for magnitude, and the status pair for pass and fail
BLUE, ORANGE = "#2a78d6", "#eb6834"
GOOD, CRITICAL = "#0ca30c", "#d03b3b"
RAMP = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5", "#2a78d6",
        "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]
LIGHT, MID, DARK = RAMP[3], RAMP[7], RAMP[11]
SEQUENTIAL = LinearSegmentedColormap.from_list("blue", RAMP)
SIM_NOTE = "PX4 v1.17.0 SITL with Gazebo and assumed vehicle coefficients; simulation, not flight data."

plt.rcParams.update({
    "font.family": "sans-serif", "font.size": 8.5,
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "text.color": INK, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "axes.titlesize": 9, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "grid.linestyle": "-",
    "axes.axisbelow": True,
    "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
    "xtick.major.size": 0, "ytick.major.size": 0,
    "lines.linewidth": 1.6, "lines.solid_capstyle": "round", "lines.solid_joinstyle": "round",
    "legend.frameon": False, "legend.fontsize": 8, "legend.handlelength": 1.6,
    "savefig.dpi": 220, "pdf.fonttype": 42,
})


def save(fig, name):
    FIGURES.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(FIGURES / f"{name}.{ext}", bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)
    print("wrote", FIGURES / f"{name}.png")


def _extent(fig):
    """The figure-fraction box around everything drawn so far: axes, their labels, legends."""
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    boxes = [a.get_tightbbox(renderer) for a in fig.axes if a.get_visible()]
    boxes += [legend.get_window_extent(renderer) for legend in fig.legends]
    boxes += [t.get_window_extent(renderer) for t in fig.texts]
    union = matplotlib.transforms.Bbox.union([b for b in boxes if b is not None])
    return union.transformed(fig.transFigure.inverted())


def heading(fig, title, subtitle):
    """The takeaway in bold, then what was flown, clear above everything drawn."""
    box = _extent(fig)
    left = min(box.x0, 0.0)
    sub = fig.text(left, box.y1 + 0.012, subtitle, fontsize=8.5, color=INK2, ha="left",
                   va="bottom", linespacing=1.35)
    lines = subtitle.count("\n") + 1
    above = matplotlib.transforms.offset_copy(sub.get_transform(), fig=fig, y=lines * 12.5 + 5,
                                              units="points")
    fig.text(left, box.y1 + 0.012, title, fontsize=11.5, fontweight="bold", color=INK, ha="left",
             va="bottom", transform=above)


def footnote(fig, text):
    """A muted line clear below everything drawn."""
    box = _extent(fig)
    fig.text(min(box.x0, 0.0), box.y0 - 0.012, text, fontsize=7.2, color=MUTED, ha="left",
             va="top", linespacing=1.35)


def load(path):
    return json.loads(Path(path).read_text())


def values_of(record):
    """The case values a record was flown with: its manifest, else the nominal plus overrides."""
    manifest = Path(record.get("manifest") or "")
    if manifest.is_file():
        return load(manifest)["values"]
    return {**CFG["nominal"], **{k: v for k, v in record.get("overrides", {}).items() if ":" not in k}}


def ulog_topics(record, topics):
    from pyulog import ULog
    ulog = ULog(record["log"], message_name_filter_list=topics)
    return {d.name: d.data for d in ulog.data_list if d.multi_id == 0}


# ---------------------------------------------------------------------------------------------
# 0. The vehicle from above: what can be controlled
# ---------------------------------------------------------------------------------------------
PROP_RADIUS = 0.19              # drawn for illustration; the propeller size is not given


def vehicle_plan():
    from matplotlib.patches import Circle, FancyArrowPatch, Patch, Rectangle
    n = CFG["nominal"]
    rx, ry = n["rotor_x_m"], n["rotor_y_m"]
    span, chord = n["wing_span_m"], n["wing_chord_m"]
    plate_te = rx - 1.065       # trailing edge of the rear plate, 1065 mm behind the front motors
    fixed = dict(facecolor=GRID, edgecolor=AXIS, lw=1.0)

    fig, ax = plt.subplots(figsize=(7.2, 6.4))
    ax.set_aspect("equal")
    ax.axis("off")
    # nose to the right: x forward, y to the left of the vehicle is up on the page
    ax.add_patch(Rectangle((-chord / 2, -span / 2), chord, span, **fixed))          # wing
    ax.add_patch(Rectangle((plate_te, -0.2375), 0.15, 0.475, **fixed))              # rear plate
    for y in (ry, -ry):
        ax.plot([plate_te + 0.075, rx], [y, y], color=AXIS, lw=3, solid_capstyle="butt")
    ax.add_patch(Rectangle((-0.355, -0.1025), 0.71, 0.205, facecolor="#d6d5cd",
                           edgecolor=AXIS, lw=1.0))                                  # body
    # PX4 motor order of a quad-X and the spin of each
    rotors = [(1, rx, -ry, "CCW"), (2, -rx, ry, "CCW"), (3, rx, ry, "CW"), (4, -rx, -ry, "CW")]
    for number, x, y, spin in rotors:
        ax.add_patch(Circle((x, y), PROP_RADIUS, facecolor=BLUE, alpha=0.12, lw=0))
        ax.add_patch(Circle((x, y), PROP_RADIUS, fill=False, edgecolor=BLUE, lw=1.6))
        ax.plot(x, y, "o", ms=5, color=BLUE)
        ax.text(x, y + (0.075 if y > 0 else -0.075), f"{number}  {spin}", ha="center",
                va="center", fontsize=8.5, color=INK)
    px = n["pusher_x_m"]
    ax.plot([px, px], [-0.13, 0.13], color=ORANGE, lw=3.5, solid_capstyle="round")
    ax.add_patch(FancyArrowPatch((px + 0.03, 0), (px + 0.2, 0), arrowstyle="-|>",
                                 mutation_scale=12, color=ORANGE, lw=1.6))
    ax.plot(0, 0, "+", ms=11, mew=1.6, color=INK)
    ax.text(0.035, -0.05, "CG", fontsize=8, color=INK)

    def dimension(a, b, text, offset=(0, 0), **kw):
        ax.annotate("", a, b, arrowprops=dict(arrowstyle="<->", color=MUTED, lw=0.8,
                                              shrinkA=0, shrinkB=0))
        ax.text((a[0] + b[0]) / 2 + offset[0], (a[1] + b[1]) / 2 + offset[1], text,
                fontsize=7.5, color=INK2, ha="center", va="center", **kw)

    dimension((0.72, -span / 2), (0.72, span / 2), "wing span 1700", (0.035, 0), rotation=90)
    dimension((-chord / 2, 0.80), (chord / 2, 0.80), "chord 250", (0, 0.035))
    dimension((-rx, -0.52), (rx, -0.52), "motors 665", (0, -0.035))
    dimension((0.60, -ry), (0.60, ry), "505", (0.035, 0), rotation=90)
    dimension((plate_te, 0.52), (rx, 0.52), "front motors to the plate's trailing edge 1065",
              (0, 0.035))
    ax.text(plate_te + 0.075, -0.30, "rear plate\n475 x 150", fontsize=7.5, color=INK2,
            ha="center", va="top", linespacing=1.1)
    ax.text(0.17, 0.06, "body 710 x 205", fontsize=7.5, color=INK2, ha="center", va="center")
    ax.text(px - 0.03, 0.17, "pusher", fontsize=8.5, color=INK, ha="center")
    ax.annotate("", (0.98, -0.78), (0.80, -0.78),
                arrowprops=dict(arrowstyle="-|>", color=INK2, lw=1.0, mutation_scale=10))
    ax.text(0.89, -0.74, "forward", fontsize=8, color=INK2, ha="center")

    keys = [Line2D([], [], marker="o", ls="", ms=9, mfc="#dbe8f8", mec=BLUE, mew=1.6,
                   label="Lift rotor: height, roll, pitch, yaw"),
            Line2D([], [], color=ORANGE, lw=3.5, label="Pusher: forward force"),
            Patch(facecolor=GRID, edgecolor=AXIS, label="Fixed: no control surfaces")]
    ax.legend(handles=keys, loc="lower left", bbox_to_anchor=(0.0, 0.02), ncol=1,
              handlelength=1.8, labelspacing=0.6)
    ax.text(0.0, -0.01, "Lengths in mm, from the airframe team. Assumed: the CG at the centre "
            "of the four motors, the wing and the body\ncentred on it, the pusher behind the body. "
            "Propellers drawn for illustration.", transform=ax.transAxes, fontsize=7.5,
            color=INK2, va="top", linespacing=1.3)
    ax.set_xlim(-0.95, 1.02)
    ax.set_ylim(-0.92, 0.92)
    ax.set_title("The vehicle from above: five things can be commanded", pad=4)
    save(fig, "fig0_vehicle")




# ---------------------------------------------------------------------------------------------
# 1. One flight: where it went and what it did
# ---------------------------------------------------------------------------------------------
def takeoff_frame(flight):
    """Maps north/east to metres (to the right, forward) of the take-off point and heading."""
    hx, hy = flight["home"][:2]
    c, s = math.cos(flight["yaw0"]), math.sin(flight["yaw0"])

    def to(north, east):
        dn, de = np.asarray(north) - hx, np.asarray(east) - hy
        return -dn * s + de * c, dn * c + de * s
    return to


def flight_parts(phases):
    """[start, end, label] for the take-off, each leg of the route, the landing and a failsafe."""
    parts = []
    for name, (a, b) in sorted(phases.items(), key=lambda item: item[1][0]):
        kind = name.rstrip("0123456789")
        number = int(name[len(kind):] or (2 if kind == "hover" else 1))
        if name in ("climb", "hover1"):
            label = "Take-off"
        elif kind in ("align", "descend", "land"):
            label = "Landing"
        elif kind in ("stop", "px4"):
            label = "Failsafe"
        else:                       # hover<i+1> ends leg i; corner<i> turns from leg i
            label = f"Leg {number - 1 if kind == 'hover' else number}"
        if parts and parts[-1][2] == label:
            parts[-1][1] = b
        else:
            parts.append([a, b, label])
    return parts


def ground_track(ax, record, color, label, planned=True):
    flight = record["flight"]
    to = takeoff_frame(flight)
    pos = ulog_topics(record, ["vehicle_local_position"])["vehicle_local_position"]
    t = pos["timestamp"] * 1e-6
    ph = flight["phases"]
    keep = (t >= ph["climb"][0]) & (t <= max(end for _, end in ph.values()))
    right, forward = to(pos["x"][keep], pos["y"][keep])
    legs = flight.get("legs") or []
    if planned and legs:
        corners = [legs[0]["start"]] + [leg["end"] for leg in legs]
        pr, pf = to([c[0] for c in corners], [c[1] for c in corners])
        ax.plot(pr, pf, color=GRID, lw=8, solid_capstyle="round", zorder=1, label="Planned route")
        for i, (r, f) in enumerate(zip(pr[1:], pf[1:]), 1):
            if math.hypot(r, f) < 1.0:          # back at the take-off point: labelled below
                continue
            ax.plot(r, f, "o", ms=15, mfc=SURFACE, mec=INK2, mew=1.0, zorder=5)
            ax.text(r, f, str(i), ha="center", va="center", fontsize=7.5, color=INK, zorder=6)
        back = math.hypot(pr[-1], pf[-1]) < 1.0
        home_text = (f"Take-off, and landing\nat waypoint {len(legs)}" if back
                     else "Take-off point")
        for (r0, f0), (r1, f1) in zip(zip(pr, pf), zip(pr[1:], pf[1:])):
            mid, step = np.array([(r0 + r1) / 2, (f0 + f1) / 2]), np.array([r1 - r0, f1 - f0])
            step = step / np.hypot(*step) * 2.5
            ax.annotate("", mid + step, mid - step, zorder=4,
                        arrowprops=dict(arrowstyle="-|>", color=INK2, lw=1.0, mutation_scale=9))
        ax.plot(0, 0, "s", ms=8, color=INK, zorder=6)
        ax.annotate(home_text, (0, 0), xytext=(9, 9), textcoords="offset points",
                    ha="left", va="bottom", fontsize=7.5, color=INK2, linespacing=1.15)
    ax.plot(right, forward, color=color, lw=1.8 if planned else 1.3, zorder=3, label=label)


def flight(record, label, compare=None, compare_label=None, name="fig1_flight"):
    """Ground track beside the time history of speed, attitude, height and motor commands."""
    f, fl, v = record["figures"], record["flight"], values_of(record)
    data = ulog_topics(record, ["vehicle_local_position", "vehicle_local_position_setpoint",
                                "vehicle_attitude", "actuator_motors"])
    pos, asked = data["vehicle_local_position"], data["vehicle_local_position_setpoint"]
    att, mot = data["vehicle_attitude"], data["actuator_motors"]
    ph = fl["phases"]
    t0, t1 = ph["climb"][0], max(end for _, end in ph.values())

    def clip(d):
        t = d["timestamp"] * 1e-6
        keep = (t >= t0) & (t <= t1)
        return t[keep] - t0, keep

    fig = plt.figure(figsize=(11.0, 6.2))
    grid = fig.add_gridspec(4, 2, width_ratios=[1.0, 1.55], wspace=0.22, hspace=0.14)
    track = fig.add_subplot(grid[:, 0])
    axes = [fig.add_subplot(grid[0, 1])]
    axes += [fig.add_subplot(grid[i, 1], sharex=axes[0]) for i in (1, 2, 3)]

    ground_track(track, record, BLUE, label)
    if compare is not None:
        ground_track(track, compare, ORANGE, compare_label, planned=False)
    track.set_aspect("equal")
    track.set_xlabel("To the right of the take-off heading (m)")
    track.set_ylabel("Forward along the take-off heading (m)")
    track.margins(0.12)
    track.legend(loc="upper center", bbox_to_anchor=(0.5, -0.11), ncol=1)

    parts = flight_parts(ph)
    turns = [(a - t0, b - t0) for n, (a, b) in ph.items() if n.startswith(("turn", "corner"))]
    for ax in axes:
        ax.grid(axis="x", visible=False)
        for i, (a, b, _) in enumerate(parts):
            if i % 2:
                ax.axvspan(a - t0, b - t0, color=BAND, lw=0, zorder=0)
        for a, b in turns:
            ax.axvspan(a, b, color=GRID, lw=0, zorder=0)
        ax.tick_params(axis="x", labelbottom=ax is axes[-1])
        ax.yaxis.set_label_coords(-0.075, 0.5)
    for a, b, text in parts:
        axes[0].text((a + b) / 2 - t0, 1.04, text, transform=axes[0].get_xaxis_transform(),
                     ha="center", va="bottom", fontsize=7.5, color=INK2)

    ax = axes[0]
    t, keep = clip(asked)
    ax.plot(t, np.hypot(np.nan_to_num(asked["vx"][keep]), np.nan_to_num(asked["vy"][keep])),
            color=MUTED, lw=1.2, label="PX4 velocity setpoint")
    t, keep = clip(pos)
    speed = np.hypot(pos["vx"][keep], pos["vy"][keep])
    ax.plot(t, speed, color=BLUE, label="Flown")
    ax.set_ylabel("Horizontal\nspeed (m/s)")
    ax.set_ylim(0, np.nanmax(speed) * 1.45)
    ax.legend(loc="upper right", ncol=2)

    ax = axes[1]
    roll, pitch, _ = metrics._euler_deg(att)
    t, keep = clip(att)
    ax.plot(t, pitch[keep], color=BLUE, label="Pitch, nose up +")
    ax.plot(t, roll[keep], color=ORANGE, label="Roll, right +")
    limit = metrics.LIMITS["tilt_max_deg"]
    peak = np.nanmax(np.abs(np.r_[roll[keep], pitch[keep]]))
    span = max(12.0, peak * 1.5)
    if peak * 1.15 >= limit:
        for sign in (1, -1):
            ax.axhline(sign * limit, color=CRITICAL, lw=0.9)
        ax.text(0.5, limit, f"pass limit ±{limit:g}°", fontsize=7, color=INK2, va="bottom")
    ax.set_ylim(-span, span)
    ax.set_ylabel("Roll, pitch (°)")
    ax.legend(loc="upper right", ncol=2)

    ax = axes[2]
    t, keep = clip(pos)
    ax.axhline(v["alt_m"], color=MUTED, lw=0.9)
    ax.plot(t, fl["home"][2] - pos["z"][keep], color=BLUE)
    ax.text(0.5, v["alt_m"], f"target {v['alt_m']:g} m", fontsize=7, color=INK2, va="bottom")
    ax.set_ylim(0, v["alt_m"] * 1.3)
    ax.set_ylabel("Height above\ntake-off (m)")

    ax = axes[3]
    t, keep = clip(mot)
    lift = np.stack([mot[f"control[{i}]"][keep] for i in range(4)])
    ax.fill_between(t, lift.min(axis=0), lift.max(axis=0), color=BLUE, alpha=0.15, lw=0)
    ax.plot(t, lift.mean(axis=0), color=BLUE, label="Lift rotors: mean, band from lowest to highest")
    ax.plot(t, np.nan_to_num(mot["control[4]"][keep]), color=ORANGE, label="Pusher")
    ax.set_ylim(0, 1.25)
    ax.set_yticks([0, 0.5, 1.0])
    ax.set_ylabel("Motor command\n(0 to 1)")
    ax.legend(loc="upper right", ncol=2)
    ax.set_xlabel("Time since the start of the climb (s)")
    ax.set_xlim(0, t1 - t0)

    outcome = "passes every check" if record["passed"] else "fails: " + "; ".join(record["reasons"])[:70]
    wind = (f"{v['wind_speed_mps']:g} m/s wind from {v['wind_from_deg']:g}° off the take-off heading"
            if v["wind_speed_mps"] > 0 else "no wind")
    footnote(fig, SIM_NOTE)
    heading(fig, f"{label}: {outcome}",
            f"{len(fl.get('legs', [])) or 1} legs at {v['cruise_mps']:g} m/s, {wind}. Landed "
            f"{f.get('touchdown_err_m', math.nan):.2f} m from the take-off point; largest tilt "
            f"{f.get('tilt_max_deg', math.nan):.0f}°.\nOn the right, alternate legs are shaded; the "
            + ("darker strips are the corners, flown banked without stopping."
               if v.get("fly_through", 0) >= 0.5 else "darker strips are the turns, flown in a hover."))
    save(fig, name)


def oscillation(clean, bad, leg, name="fig6_sideways_oscillation"):
    """Roll and sideways speed on one leg, for a calm run and for the run that oscillated."""
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 4.4), sharex=True, gridspec_kw={"hspace": 0.12})
    for record, color, text in ((clean, BLUE, "Flown alone"), (bad, ORANGE, "Flown beside other simulations")):
        fl = record["flight"]
        heading_rad = fl["legs"][leg - 1]["heading"]
        start, brake = fl["phases"][f"accel{leg}"][0], fl["phases"][f"decel{leg}"][0]
        end = fl["phases"][f"hover{leg + 1}"][0] + 2.0
        d = ulog_topics(record, ["vehicle_attitude", "vehicle_local_position"])
        att, pos = d["vehicle_attitude"], d["vehicle_local_position"]
        roll, _, _ = metrics._euler_deg(att)
        ta, tp = att["timestamp"] * 1e-6, pos["timestamp"] * 1e-6
        ka, kp = (ta >= start) & (ta <= end), (tp >= start) & (tp <= end)
        side = -pos["vx"][kp] * math.sin(heading_rad) + pos["vy"][kp] * math.cos(heading_rad)
        axes[0].plot(ta[ka] - brake, roll[ka], color=color, label=text)
        axes[1].plot(tp[kp] - brake, side, color=color, label=text)
    for ax in axes:
        ax.axvline(0, color=INK2, lw=0.9)
        ax.grid(axis="x", visible=False)
    axes[0].text(0.15, 0.95, "slow-down starts,\npusher cuts out", transform=axes[0].get_xaxis_transform(),
                 fontsize=7.2, color=INK2, va="top")
    limit = metrics.LIMITS["tilt_max_deg"]
    for sign in (1, -1):
        axes[0].axhline(sign * limit, color=CRITICAL, lw=0.9)
    axes[0].text(axes[0].get_xlim()[0] + 0.2, -limit, f"pass limit ±{limit:g}°", fontsize=7,
                 color=INK2, va="bottom")
    for ax in axes:
        low, high = ax.get_ylim()
        ax.set_ylim(low, high + (high - low) * 0.25)
    axes[0].set_ylabel("Roll (°)")
    axes[1].set_ylabel(f"Speed across\nleg {leg} (m/s)")
    axes[1].set_xlabel("Time from the start of the slow-down (s)")
    axes[0].legend(loc="upper left", ncol=2, bbox_to_anchor=(0.0, 1.0))
    footnote(fig, SIM_NOTE)
    peak = bad["figures"]["tilt_max_deg"]
    heading(fig, f"A run that shared the machine with other simulations swung into a {peak:.0f}° roll; "
            "runs alone never did",
            f"Leg {leg} of the 50 m square at 5 m/s, no wind, the same profile in both. A sway of about 1 Hz "
            "appeared 1.5 s before braking and peaked\nonce the pusher cut out. Beside other simulations "
            "6 of 11 squares wobbled like this, alone 0 of 13: Gazebo does not wait for PX4,\nso a busy "
            "machine delays the motor commands. It is an artifact of the simulation, not of the vehicle.")
    save(fig, name)


def braking(groups, name="fig7_braking"):
    """Commanded speed, pitch and pusher through every slow-down, for each version of the profile.

    groups: [(label, records)], drawn in that order; the last is the current profile.
    """
    grid = np.linspace(-1.0, 10.0, 221)
    fig, axes = plt.subplots(3, 1, figsize=(7.4, 6.0), sharex=True, gridspec_kw={"hspace": 0.12})
    keys, peaks = [], []
    for (label, records), color in zip(groups, (ORANGE, BLUE)):
        curves = {"speed": [], "pitch": [], "pusher": []}
        for record in records:
            d = ulog_topics(record, ["vehicle_attitude", "vehicle_local_position_setpoint",
                                     "actuator_motors"])
            att, sp, mot = d["vehicle_attitude"], d["vehicle_local_position_setpoint"], d["actuator_motors"]
            _, pitch, _ = metrics._euler_deg(att)
            series = {"speed": (sp["timestamp"], np.hypot(np.nan_to_num(sp["vx"]), np.nan_to_num(sp["vy"]))),
                      "pitch": (att["timestamp"], pitch),
                      "pusher": (mot["timestamp"], np.nan_to_num(mot["control[4]"]))}
            for phase, (start, _) in record["flight"]["phases"].items():
                if phase.rstrip("0123456789") != "decel":
                    continue
                for key, (stamps, values) in series.items():
                    curves[key].append(np.interp(grid, stamps * 1e-6 - start, values,
                                                 left=np.nan, right=np.nan))
        for ax, key in zip(axes, curves):
            rows = np.array(curves[key])
            for row in rows:
                ax.plot(grid, row, color=color, lw=0.6, alpha=0.22, zorder=2)
            ax.plot(grid, np.nanmedian(rows, axis=0), color=color, lw=2.0, zorder=3)
        peaks.append(np.median(np.nanmax(np.array(curves["pitch"]), axis=1)))
        keys.append(Line2D([], [], color=color, lw=2.0,
                           label=f"{label}: median of {len(curves['pitch'])} slow-downs, thin lines each"))
    for ax in axes:
        ax.axvline(0, color=INK2, lw=0.9)
        ax.grid(axis="x", visible=False)
    axes[0].set_ylabel("Commanded\nspeed (m/s)")
    axes[1].set_ylabel("Pitch, nose\nup (°)")
    axes[2].set_ylabel("Pusher\ncommand")
    axes[2].set_xlabel("Time from the start of the slow-down (s)")
    axes[0].set_ylim(0, None)
    low, high = axes[1].get_ylim()
    axes[1].set_ylim(low, high + (high - low) * 0.15)
    axes[0].legend(handles=keys, loc="upper right", fontsize=7.4)
    footnote(fig, SIM_NOTE)
    heading(fig, f"Easing into the braking lowers the nose-up from {peaks[0]:.1f}° to {peaks[-1]:.1f}° "
            "and lets the pusher fade out",
            "Every slow-down of the 50 m square at 5 m/s, no wind, each run flown with no other simulation "
            "on the machine.\nThe eased braking takes longer (7.9 s instead of 5 s from 5 m/s) and is "
            "never harder than the old one.")
    save(fig, name)


def _route_s(record):
    """Seconds from the first speed-up to the hover over the last waypoint."""
    ph = record["flight"]["phases"]
    last = max((name for name in ph if name.startswith("hover")), key=lambda n: int(n[5:]))
    return ph[last][0] - ph["accel1"][0]


def speeds(records, name="fig9_speeds"):
    """What the cruise speed buys and costs on the square, in each route mode (no wind, no event)."""
    modes = [(1, "Flown through the corners", BLUE, "o"), (0, "Stopped at each corner", ORANGE, "s")]
    panels = [("Time from the first speed-up\nto the last waypoint (s)", _route_s),
              ("Fastest flown (m/s)", lambda r: r["figures"].get("true_airspeed_max_mps", math.nan)),
              ("Largest roll or pitch\nin forward flight (°)",
               lambda r: max(r["figures"].get("fwd_roll_max_deg", math.nan),
                             r["figures"].get("fwd_pitch_max_deg", math.nan))),
              ("Farthest from the\nplanned path (m)", lambda r: r["figures"].get("cross_track_max_m", math.nan))]
    fig, axes = plt.subplots(2, 2, figsize=(7.4, 5.2), sharex=True,
                             gridspec_kw={"hspace": 0.32, "wspace": 0.34})
    calm = [r for r in records if not r.get("event") and "wind_speed_mps" not in r["overrides"]
            and r["figures"]]
    for ax, (ylabel, value) in zip(axes.flat, panels):
        for through, label, color, mark in modes:
            rows = sorted((r["overrides"]["cruise_mps"], value(r)) for r in calm
                          if r["overrides"].get("fly_through") == through)
            x, y = zip(*rows)
            ax.plot(x, y, color=color, marker=mark, ms=5, label=label,
                    markeredgecolor=SURFACE, markeredgewidth=1.0)
        ax.set_ylabel(ylabel)
        ax.set_ylim(0, None)
        ax.grid(axis="x", visible=False)
    axes[0, 1].plot([3, 7], [3, 7], color=MUTED, lw=0.9, ls=(0, (3, 2)))
    axes[0, 1].text(4.6, 4.3, "planned cruise", fontsize=7, color=INK2, ha="left", va="top")
    for ax in axes[1]:
        ax.set_xlabel("Planned cruise speed (m/s)")
        ax.set_xticks([3, 4, 5, 6, 7])
    axes[0, 0].legend(loc="center right")
    fails = [r["case"] for r in calm if not r["passed"]]
    footnote(fig, SIM_NOTE + ("" if not fails else "  Failed: " + ", ".join(fails)))
    through = {r["overrides"]["cruise_mps"]: r for r in calm if r["overrides"].get("fly_through") == 1}
    stop = {r["overrides"]["cruise_mps"]: r for r in calm if r["overrides"].get("fly_through") == 0}
    heading(fig, f"Flying through the corners halves the square ({_route_s(through[5.0]):.0f} s against "
            f"{_route_s(stop[5.0]):.0f} s at 5 m/s); above 5 m/s it gains little",
            "The 50 m square, no wind, each run alone. The corners of 15 m radius are taken at most "
            "sqrt(1.7 m/s² × 15 m) = 5 m/s,\nso a faster cruise is only reached on the first and last "
            "leg, and the middle legs speed up and slow down without a cruise.")
    save(fig, name)


def silence(runs, name="fig10_silence"):
    """Pitch and speed after the setpoints stop in cruise, for each way of sending them.

    runs: [(label, record)], each a run with a silence event; the last is the current one.
    """
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 4.6), sharex=True, gridspec_kw={"hspace": 0.12})
    peaks = []
    for (label, record), color in zip(runs, (ORANGE, BLUE)):
        fl = record["flight"]
        t0 = fl["event"]["t"]
        d = ulog_topics(record, ["vehicle_attitude", "vehicle_local_position"])
        att, pos = d["vehicle_attitude"], d["vehicle_local_position"]
        _, pitch, _ = metrics._euler_deg(att)
        ta, tp = att["timestamp"] * 1e-6 - t0, pos["timestamp"] * 1e-6 - t0
        ka, kp = (ta >= -1) & (ta <= 8), (tp >= -1) & (tp <= 8)
        axes[0].plot(ta[ka], pitch[ka], color=color, label=label)
        axes[1].plot(tp[kp], np.hypot(pos["vx"][kp], pos["vy"][kp]), color=color, label=label)
        hold = fl["phases"]["px4"][0] - t0
        for ax in axes:
            ax.axvline(hold, color=color, lw=0.9, ls=(0, (3, 2)))
        peaks.append(float(np.nanmax(np.abs(pitch[ka]))))
    limit = metrics.LIMITS["tilt_max_deg"]
    axes[0].axhline(limit, color=CRITICAL, lw=0.9)
    axes[0].text(7.9, limit, f"pass limit {limit:g}°", fontsize=7, color=INK2, va="bottom", ha="right")
    axes[1].text(7.9, 4.6, "dashed: PX4 calls the link lost and holds", fontsize=7, color=INK2,
                 ha="right", va="top")
    for ax in axes:
        ax.axvline(0, color=INK2, lw=0.9)
        ax.grid(axis="x", visible=False)
    axes[0].text(0.08, 0.95, "last setpoint", transform=axes[0].get_xaxis_transform(), fontsize=7.2,
                 color=INK2, va="top")
    axes[0].set_ylim(-6, limit + 6)
    axes[0].set_ylabel("Pitch, nose up (°)")
    axes[1].set_ylabel("Horizontal\nspeed (m/s)")
    axes[1].set_ylim(0, None)
    axes[1].set_xlabel("Time since the last setpoint (s)")
    axes[0].legend(loc="center right")
    footnote(fig, SIM_NOTE)
    heading(fig, f"When the Jetson goes silent at 5 m/s, the last setpoint decides: {peaks[0]:.0f}° "
            f"nose-up with its point in it, {peaks[-1]:.0f}° without",
            "The 50 m square, no wind; the setpoints stop 1.5 s into the second leg. PX4 calls the link "
            "lost after 1 s (COM_OF_LOSS_T)\nand holds. Until then it flies the last setpoint: with a "
            "point, it brakes back to a point already passed; with only a velocity, it flies on.")
    save(fig, name)


# ---------------------------------------------------------------------------------------------
# 2. Which steady winds the vehicle handles
# ---------------------------------------------------------------------------------------------
# Why a case failed, most severe first. The marker shape carries the reason; colour only pass/fail.
REASONS = [("tilt", "Tilted past 30°", "D"),
           ("pusher", "Pusher ran out (saturated, or could not reach speed)", "^"),
           ("heading", "Heading error past 20°", "s"),
           ("other", "Another check failed", "X")]


def reason(record):
    text = " ".join(record["reasons"])
    if "tilt_max" in text or "tumbled" in text:
        return "tilt"
    if "pusher" in text or "timeout in accel" in text:
        return "pusher"
    if "yaw_err" in text:
        return "heading"
    return "other"


def wind_cases(records):
    """{(speed, from_deg): record}, with the firm heading hold where the study has two."""
    cases = {}
    for r in records:
        parts = r["case"].split("_")
        if not parts[0].startswith("wind") or (len(parts) > 2 and parts[2] != "firm"):
            continue
        speed, direction = float(parts[0][4:]), float(parts[1][4:])
        cases[(speed, direction)] = r
    return cases


def wind_envelope(sets, name="fig2_wind_envelope"):
    directions = [0, 45, 90, 135, 180, 225, 270, 315]
    names = {0: "head", 90: "left", 180: "tail", 270: "right"}
    fig, axes = plt.subplots(1, len(sets), figsize=(4.3 * len(sets), 3.5), sharey=True,
                             gridspec_kw={"wspace": 0.08})
    axes = np.atleast_1d(axes)
    counts = []
    for ax, (label, records) in zip(axes, sets):
        cases = wind_cases(records)
        speeds = sorted({s for s, _ in cases})
        passed = sum(r["passed"] for r in cases.values())
        counts.append((label, passed, len(cases),
                       sum(r["passed"] for (s, _), r in cases.items() if s >= 6),
                       sum(1 for s, _ in cases if s >= 6)))
        for (speed, direction), r in cases.items():
            x, y = directions.index(int(direction)), speeds.index(speed)
            if r["passed"]:
                ax.plot(x, y, "o", ms=11, color=GOOD, mec=SURFACE, mew=2)
            else:
                marker = next(m for key, _, m in REASONS if key == reason(r))
                ax.plot(x, y, marker, ms=10, color=CRITICAL, mec=SURFACE, mew=2)
            if r.get("rerun_alone"):
                ax.plot(x, y, "o", ms=17, mfc="none", mec=INK, mew=1.1)
        ax.set_xticks(range(len(directions)),
                      [f"{d}°\n{names[d]}" if d in names else f"{d}°" for d in directions])
        ax.set_yticks(range(len(speeds)), [f"{s:g}" for s in speeds])
        ax.set_xlim(-0.6, len(directions) - 0.4)
        ax.set_ylim(-0.6, len(speeds) - 0.4)
        ax.grid(False)
        ax.spines["left"].set_visible(False)
        ax.set_title(f"{label}\n{passed} of {len(cases)} pass", fontsize=8.5, linespacing=1.4)
        ax.set_xlabel("Wind from, degrees off the take-off heading")
    axes[0].set_ylabel("Steady wind (m/s)")
    keys = [Line2D([], [], marker="o", ls="", ms=9, color=GOOD, label="Pass")]
    keys += [Line2D([], [], marker=m, ls="", ms=8, color=CRITICAL, label=f"Fail: {text.lower()}")
             for _, text, m in REASONS]
    if any(r.get("rerun_alone") for _, records in sets for r in records):
        keys.append(Line2D([], [], marker="o", ls="", ms=12, mfc="none", mec=INK, mew=1.1,
                           label="Flown again with no other simulation running; this result replaces the sweep's"))
    below = _extent(fig)
    fig.legend(handles=keys, loc="upper left", bbox_to_anchor=(below.x0, below.y0 - 0.01),
               ncol=2 if len(sets) == 1 else 3, fontsize=7.6)
    if len(counts) == 2 and counts[1][3] > counts[0][3]:
        title = (f"Flying slower handles more of the strong wind: at 6–8 m/s, {counts[0][3]} of "
                 f"{counts[0][4]} cases pass at {counts[0][0].split(',')[0]}, {counts[1][3]} of "
                 f"{counts[1][4]} at {counts[1][0].split(',')[0]}")
    else:
        title = "Which steady winds the vehicle handles"
    footnote(fig, SIM_NOTE)
    heading(fig, title, "Each mark is one full simulated flight in a steady wind (firm heading hold). "
            "Direction is measured from the nose at take-off;\non the square the vehicle meets the "
            "wind from all four sides once per lap.")
    save(fig, name)


# ---------------------------------------------------------------------------------------------
# 3. Why a headwind is the limit: pusher thrust needed against thrust available
# ---------------------------------------------------------------------------------------------
def pusher_thrust(sets, name="fig3_pusher_thrust"):
    n = CFG["nominal"]
    t_max, v_zero = n["pusher_thrust_max_n"], n["pusher_zero_thrust_mps"]

    def available(airspeed, command):
        """The simulation's pusher model (AeroPhysics.hh): static T_max*u, falling with inflow."""
        return t_max * command * np.clip(1 - np.maximum(airspeed, 0) / (v_zero * math.sqrt(command)), 0, 1)

    def straight(record):       # one heading for the whole cruise, so one inflow per flight
        return len(record["flight"].get("legs") or [None]) == 1

    def inflow(record, speed):
        """Air coming into the pusher along the nose: ground speed plus the headwind part."""
        o = record.get("overrides", {})
        return speed + o.get("wind_speed_mps", 0.0) * math.cos(math.radians(o.get("wind_from_deg", 0.0)))

    fig, ax = plt.subplots(figsize=(7.4, 4.3))
    v = np.linspace(0, 26, 300)
    ax.plot(v, available(v, 1.0), color=MUTED, lw=1.3, label="Available at full command")
    ax.plot(v, available(v, 0.9), color=INK, lw=2.0, label="Available at 0.9, PX4's cap on the pusher")
    every, drawn = [], []
    for (label, records), color in zip(sets, (BLUE, ORANGE)):
        flights = [r for r in records if straight(r)]
        held = [(inflow(r, r["figures"]["cruise_speed_mps"]), r["figures"]["cruise_pusher_cmd"])
                for r in flights if math.isfinite(r["figures"].get("cruise_pusher_cmd", math.nan))]
        stuck = [r["figures"]["true_airspeed_max_mps"]
                 for r in flights if "timeout in accel" in " ".join(r["reasons"])]
        if not held:
            continue
        drawn.append(label)
        a = np.array([p[0] for p in held])
        need = np.array([available(*p) for p in held])
        every += list(zip(a, need))
        ax.plot(a, need, "o", ms=6.5, color=color, mec=SURFACE, mew=1.5, zorder=4,
                label=f"Needed to hold cruise: {label}")
        if stuck:
            s = np.array(stuck)
            ax.plot(s, available(s, 0.9), "^", ms=7.5, color=CRITICAL, mec=SURFACE, mew=1.5, zorder=5,
                    label="Pinned at the cap, never reached cruise speed (fastest airspeed reached)")
    a, need = np.array(every).T
    fit = np.polyfit(a, need, 2)
    vv = np.linspace(max(0, a.min() - 1), 22, 200)
    ax.plot(vv, np.polyval(fit, vv), color=INK2, lw=1.0, ls=(0, (4, 2)), zorder=3,
            label="Needed, quadratic fit to the dots")
    gap = np.polyval(fit, vv) - available(vv, 0.9)
    limit = vv[np.argmax(gap > 0)] if (gap > 0).any() else math.nan
    if math.isfinite(limit):
        ax.axvline(limit, color=CRITICAL, lw=0.9)
        ax.text(limit + 0.3, 25.5, f"pusher limit:\nabout {limit:.0f} m/s of air\ninto the pusher",
                fontsize=7.5, color=INK2, va="top", linespacing=1.2)
    for speed, text in ((13, "5 m/s cruise\n+ 8 m/s headwind"), (18, "10 m/s cruise\n+ 8 m/s headwind")):
        ax.plot([speed, speed], [0, 1.2], color=INK2, lw=1.0)
        ax.text(speed, 1.6, text, fontsize=7, color=INK2, ha="center", va="bottom", linespacing=1.1)
    ax.set_xlim(0, 26)
    ax.set_ylim(0, 26)
    ax.set_xlabel("Air coming into the pusher: ground speed + headwind (m/s)")
    ax.set_ylabel("Pusher thrust (N)")
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, -0.13), ncol=2, fontsize=7.4, columnspacing=1.5)
    title = (f"In a headwind the pusher runs out at about {limit:.0f} m/s of airspeed"
             if math.isfinite(limit) else "Pusher thrust needed against thrust available")
    footnote(fig, SIM_NOTE + " Battery sag (under 2% here) is ignored.")
    heading(fig, title,
            f"The {t_max:g} N pusher gives less as the air comes in faster (none at {v_zero:g} m/s at full "
            "speed). Each dot is one simulated flight in a steady\nwind: the thrust PX4 used to hold "
            f"cruise, from its pusher command and the same model. Straight flights only ({', '.join(drawn)}).")
    save(fig, name)
    return limit


# ---------------------------------------------------------------------------------------------
# 4. Which uncertainties use up the margins
# ---------------------------------------------------------------------------------------------
PARAM_NAMES = {
    "mass_kg": ("Mass", "kg"), "inertia_scale": ("Inertia ×", ""), "cg_x_m": ("CG fore-aft", "m"),
    "cg_y_m": ("CG sideways", "m"), "cg_z_m": ("CG up-down", "m"),
    "lift_thrust_max_n": ("Lift motor max thrust", "N"), "lift_moment_m": ("Lift rotor torque ratio", "m"),
    "lift_tau_up_s": ("Lift motor time constant", "s"), "lift_mismatch_pct": ("Lift motor mismatch", "%"),
    "pusher_thrust_max_n": ("Pusher max thrust", "N"), "pusher_y_m": ("Pusher line sideways", "m"),
    "pusher_z_m": ("Pusher line up-down", "m"), "wing_incidence_deg": ("Wing incidence", "°"),
    "wing_cd0": ("Wing drag coefficient", ""), "wing_cp_x_m": ("Wing lift position", "m"),
    "wing_cm0": ("Wing pitching moment", ""), "lift_drag_at_full_rpm": ("Rotor crossflow drag", ""),
    "pusher_zero_thrust_mps": ("Pusher zero-thrust speed", "m/s"),
    "front_rotor_efficiency": ("Front rotor efficiency", ""), "rear_rotor_efficiency": ("Rear rotor efficiency", ""),
    "body_side_cd": ("Body side drag coefficient", ""), "motor_voltage_end_ratio": ("Final voltage ratio", ""),
    "wind_vertical_gust_mps": ("Vertical gusts", "m/s"), "tail_arm_m": ("Rear plate distance", "m"),
    "tail_incidence_deg": ("Rear plate incidence", "°"), "side_area_m2": ("Side area", "m²"),
    "side_cp_x_m": ("Side force position", "m"), "air_density": ("Air density", "kg/m³"),
    "wind_gust_pct": ("Gusts", "× mean"), "cruise_mps": ("Cruise speed", "m/s"),
    "accel_mps2": ("Acceleration limit", "m/s²"),
}
CRITERIA = [("Tilt", "tilt_max_deg"), ("Pusher\nsaturation", "pusher_sat_frac"),
            ("Lift motors\nat maximum", "lift_sat_frac"), ("Heading\nerror", "fwd_yaw_err_max_deg"),
            ("Off the\nleg line", "cross_track_max_m"), ("Cruise\nspeed error", "cruise_speed_err_mps"),
            ("Height\nerror", "alt_err_max_m"), ("Wing\nshare", "wing_lift_fraction_max"),
            ("Touchdown\nspeed", "touchdown_speed_mps")]


def case_label(record):
    o = {k: v for k, v in record["overrides"].items() if ":" not in k}
    if not o:
        return "Nominal vehicle"
    if "wind_from_deg" in o and len(o) == 2:
        return f"Wind {o['wind_speed_mps']:g} m/s from {o['wind_from_deg']:g}°"
    name, value = next((k, v) for k, v in o.items() if k != "wind_speed_mps" or len(o) == 1)
    text, unit = PARAM_NAMES.get(name, (name, ""))
    extra = " (5 m/s wind)" if name == "wind_gust_pct" else ""
    if name == "wind_gust_pct":
        return f"{text} {value:.0%} of the mean{extra}"
    gap = "" if unit in ("°", "%", "") else " "
    return f"{text} {value:g}{gap}{unit}"


def margins(records, name="fig4_margins", rows_shown=22):
    """Share of each pass limit used, for the cases that use the most of any."""
    limits = metrics.LIMITS
    unfinished = lambda r: any(w in " ".join(r["reasons"]) for w in ("timeout", "not completed", "tumbled"))
    table = []
    for r in records:
        use = [r["figures"].get(key, math.nan) / limits[key] for _, key in CRITERIA]
        table.append((case_label(r), np.array(use, dtype=float), unfinished(r), r))
    nominal = next(row for row in table if row[0] == "Nominal vehicle")
    others = [row for row in table if row is not nominal]
    # Touchdown speed takes a large share in every case, so it would crowd the ranking; it
    # still counts when it fails.
    sorting = [j for j, (_, key) in enumerate(CRITERIA) if key != "touchdown_speed_mps"]

    def worst(row):
        if row[2]:
            return math.inf
        return np.nanmax(row[1]) if np.nanmax(row[1]) > 1 else np.nanmax(row[1][sorting])
    others.sort(key=worst, reverse=True)
    quiet = [row for row in others if not row[2] and np.nanmax(row[1]) <= 1
             and np.nanmax(np.abs(row[1] - nominal[1])[sorting]) < 0.05]
    shown = [row for row in others if row not in quiet][:rows_shown]
    rows = [nominal] + shown

    fig, ax = plt.subplots(figsize=(8.6, 0.27 * len(rows) + 1.6))
    data = np.array([row[1] for row in rows])
    ax.imshow(np.clip(data, 0, 1), cmap=SEQUENTIAL, vmin=0, vmax=1, aspect="auto")
    for i, (label, use, dnf, r) in enumerate(rows):
        if dnf:
            ax.add_patch(plt.Rectangle((-0.5, i - 0.5), len(CRITERIA), 1, color=GRID, zorder=2))
            why = next((w for w in r["reasons"] if "timeout" in w or "tumbled" in w), r["reasons"][0])
            ax.text(len(CRITERIA) / 2 - 0.5, i, f"✕  did not finish the flight: {why}", ha="center",
                    va="center", fontsize=7.2, color=CRITICAL, zorder=3)
            continue
        for j, u in enumerate(use):
            if not math.isfinite(u):
                continue
            if u > 1:
                ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, color=CRITICAL, zorder=2))
                ax.text(j, i, f"✕ {u:.0%}", ha="center", va="center", fontsize=6.8, color="white",
                        fontweight="bold", zorder=3)
            elif u >= 0.5:
                ax.text(j, i, f"{u:.0%}", ha="center", va="center", fontsize=6.8,
                        color="white" if u > 0.65 else INK, zorder=3)
    ax.axhline(0.5, color=SURFACE, lw=3)
    ax.set_xticks(range(len(CRITERIA)), [c for c, _ in CRITERIA], fontsize=7.2)
    ax.xaxis.tick_top()
    ax.set_yticks(range(len(rows)), [row[0] for row in rows], fontsize=7.4)
    ax.grid(False)
    for spine in ax.spines.values():
        spine.set_visible(False)
    bar = fig.colorbar(ax.images[0], ax=ax, fraction=0.03, pad=0.02, ticks=[0, 0.5, 1])
    bar.ax.set_yticklabels(["0", "half", "at the limit"], fontsize=7)
    bar.outline.set_visible(False)
    column = [k for _, k in CRITERIA].index("touchdown_speed_mps")
    td = np.array([row[1][column] for row in table if not row[2]])
    footnote(fig, SIM_NOTE)
    heading(fig, "Wind and the pusher use up the most margin; every landing takes a large share of the "
            "touchdown limit",
            "Each row is one flight with one value at the low or high end of its range "
            "(10 m/s straight mission). A cell shows how much of that check's\npass limit the flight "
            f"used; red cells went past it. Numbers only from half the limit up. Touchdown speed used "
            f"{np.nanmin(td):.0%} to {np.nanmax(td):.0%} of its limit\n({nominal[1][column]:.0%} "
            f"nominal), so rows are ranked by the other checks. {len(quiet)} more cases stayed within "
            "5 points of the nominal and are not shown.")
    save(fig, name)


# ---------------------------------------------------------------------------------------------
# 5. How much of the weight the wing carries
# ---------------------------------------------------------------------------------------------
def wing_share_curve():
    airspeed = np.linspace(2, 22, 200)
    fig, ax = plt.subplots(figsize=(5.4, 3.5))
    n = CFG["nominal"]

    def share_at(speed, incidence):
        p = dict(n, wing_incidence_deg=incidence)
        return gz_model.derived(p)["wing_lift_fraction_cruise"] * (speed / p["cruise_mps"]) ** 2
    for incidence, color in zip((0.0, 2.0, 4.0), (LIGHT, MID, DARK)):
        share = share_at(airspeed, incidence)
        assumed = " (assumed for the vehicle)" if incidence == n["wing_incidence_deg"] else ""
        ax.plot(airspeed, share, color=color, label=f"{incidence:g}°{assumed}")
        ax.annotate(f"{incidence:g}°", (airspeed[-1], min(share[-1], 1.17)), xytext=(4, 0),
                    textcoords="offset points", va="center", fontsize=7.5, color=INK2,
                    annotation_clip=False)
    ax.axhline(0.4, color=CRITICAL, lw=0.9)
    ax.text(2.3, 0.425, "40%: the check used in simulation", fontsize=7.5, color=INK2)
    for speed, text in ((5, "5 m/s\nsquare"), (10, "10 m/s\nstraight")):
        ax.axvline(speed, color=AXIS, lw=0.9)
        ax.text(speed, 1.13, text, fontsize=7, color=INK2, ha="center", va="top", linespacing=1.1)
    ax.set_ylim(0, 1.2)
    ax.set_xlim(2, 22)
    ax.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(1.0))
    ax.set_xlabel("Airspeed in level flight (m/s)")
    ax.set_ylabel("Share of the weight on the wing")
    ax.legend(loc="upper left", title="Wing incidence", title_fontsize=8, bbox_to_anchor=(0.01, 0.83))
    i0 = n["wing_incidence_deg"]
    heading(fig, f"At 5 m/s the wing carries {share_at(5.0, i0):.0%} of the weight; the lift rotors "
            "carry the rest",
            f"Level flight, {n['mass_kg']:g} kg, {n['wing_span_m']:g} × {n['wing_chord_m']:g} m "
            f"NACA 4415 wing, assumed coefficients. At {i0:g}° incidence the share is "
            f"{share_at(10.0, i0):.0%} at 10 m/s\nand {share_at(20.0, i0):.0%} at 20 m/s. When "
            "braking, the nose comes up and the share rises above these curves.")
    save(fig, "fig5_wing_share")


def route_plan(name="fig11_route_plan"):
    """The planned route of the mission file: its path, curvature, speed and accelerations."""
    from psi_vtol.profile import Profile
    p = config.profile()
    run = Profile(p, (0.0, 0.0, 0.0), 0.0).runs[0]       # facing north: forward is north
    path, plan = run["path"], run["plan"]
    t = np.linspace(0.0, plan.duration, 4000)
    s, v, a = np.array([plan.state(x)[:3] for x in t]).T
    curv = np.array([path.at(x)[3] for x in s])
    lateral = v * v * curv
    with np.errstate(divide="ignore"):
        corner_limit = np.where(curv > 1e-6, np.sqrt(p["lateral_mps2"] / curv), np.nan)

    fig = plt.figure(figsize=(11.0, 5.6))
    cols = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.5], wspace=0.22)
    track = fig.add_subplot(cols[0])
    rows = cols[1].subgridspec(3, 1, hspace=0.14)
    axes = [fig.add_subplot(rows[0])]
    axes += [fig.add_subplot(rows[i], sharex=axes[0]) for i in (1, 2)]

    # the square from above: forward up, right to the right (route_m is forward, left)
    corners = [(0.0, 0.0)] + [(f_, -l_) for f_, l_ in p["route_m"]]
    fwd, right = zip(*corners)
    track.plot(right, fwd, color=AXIS, lw=1.0, ls=(0, (3, 2)), label="Lines between the waypoints")
    pts = np.array(path.polyline(0.2))
    track.plot(pts[:, 1], pts[:, 0], color=BLUE, lw=1.8, label="Planned path")
    start = 0.0
    for kind, _, length, *_ in path.sections:
        if kind == "turn":
            for x in (start, start + length):
                n, e, _, _ = path.at(x)
                track.plot(e, n, "o", ms=4.5, color=BLUE, markeredgecolor=SURFACE, markeredgewidth=1.0)
        start += length
    for i, (f_, r_) in enumerate(corners[1:-1], 1):
        track.plot(r_, f_, "o", ms=13, color=SURFACE, markeredgecolor=INK2, markeredgewidth=1.0, zorder=5)
        track.text(r_, f_, str(i), ha="center", va="center", fontsize=7.5, color=INK, zorder=6)
    track.plot(0, 0, "s", ms=7, color=INK, zorder=6)
    track.annotate("Take-off and landing", (0, 0), xytext=(8, 8), textcoords="offset points",
                   fontsize=7.5, color=INK2)
    middle = path.at(path.sections[0][2] + path.sections[1][2] / 2)
    track.annotate(f"A corner of {p['turn_radius_m']:g} m radius,\nentered and left at the dots",
                   (middle[1], middle[0]), xytext=(28, -36), textcoords="offset points", ha="left",
                   fontsize=7.5, color=INK2, arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    track.set_aspect("equal")
    track.set_xlabel("To the right of the take-off heading (m)")
    track.set_ylabel("Forward along the take-off heading (m)")
    track.margins(0.12)
    track.legend(loc="center", bbox_to_anchor=(0.5, 0.42))

    for ax in axes:                         # the corners, shaded on every panel
        for label, _, t0, _, _, duration, _ in plan.pieces:
            if label == "corner":
                ax.axvspan(t0, t0 + duration, color=GRID, lw=0, zorder=0)
        ax.grid(axis="x", visible=False)
        ax.tick_params(axis="x", labelbottom=ax is axes[-1])
        ax.yaxis.set_label_coords(-0.07, 0.5)

    ax = axes[0]
    ax.plot(t, curv, color=BLUE)
    ax.axhline(1 / p["turn_radius_m"], color=MUTED, lw=0.9, ls=(0, (3, 2)))
    ax.text(plan.duration, 1 / p["turn_radius_m"], f"1 / {p['turn_radius_m']:g} m", fontsize=7,
            color=INK2, ha="right", va="bottom")
    ax.set_ylim(0, 1.45 / p["turn_radius_m"])
    ax.set_ylabel("Curvature\n(1/m)")

    ax = axes[1]
    ax.plot(t, corner_limit, color=MUTED, lw=1.0, ls=(0, (3, 2)),
            label=f"Corner limit: sqrt({p['lateral_mps2']:g} m/s² / curvature)")
    ax.plot(t, v, color=BLUE, label="Planned speed")
    ax.set_ylim(0, p["cruise_mps"] * 1.45)
    ax.set_ylabel("Speed\n(m/s)")
    ax.legend(loc="lower center", ncol=2)

    ax = axes[2]
    ax.plot(t, a, color=BLUE, label="Along the path")
    ax.plot(t, lateral, color=ORANGE, label="Sideways, into the corner")
    for level, text in ((p["lateral_mps2"], "sideways limit"), (p["accel_mps2"], "along limit"),
                        (-p["accel_mps2"], "along limit")):
        ax.axhline(level, color=MUTED, lw=0.9, ls=(0, (3, 2)))
        ax.text(plan.duration * 0.985, level, text, fontsize=7, color=INK2, ha="right",
                va="bottom" if level > 0 else "top")
    ax.set_ylim(-1.6, 3.0)
    ax.set_ylabel("Acceleration\n(m/s²)")
    ax.set_xlabel("Time from the first speed-up (s)")
    ax.set_xlim(0, plan.duration)
    ax.legend(loc="upper center", ncol=2)

    footnote(fig, "Computed from missions/2026_year_end/mission.yaml by psi_vtol/trajectory.py; "
             "no simulation involved. Darker strips: the corners.")
    heading(fig, f"The route is planned before take-off: straight legs, {p['turn_radius_m']:g} m corners "
            "the bank eases into, and no step in speed",
            f"The default square at {p['cruise_mps']:g} m/s. Each corner's curvature rises and falls over "
            f"{max(1.0, p['cruise_mps'] * p['brake_ramp_s']):g} m, so the vehicle banks in gradually. "
            f"Every speed change builds its acceleration up\nand takes it off over {p['brake_ramp_s']:g} s "
            f"(at most {p['accel_mps2']:g} m/s² along, {p['lateral_mps2']:g} m/s² sideways), and the speed "
            "reaches zero exactly on the last waypoint.")
    save(fig, name)


def main():
    global CFG, FIGURES
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--mission", default=config.DEFAULT_MISSION, help="folder under missions/")
    parser.add_argument("--model-revision", default=gz_model.MODEL_REVISION,
                        help="subfolder of figures/ to write into")
    parser.add_argument("--flight", help="a run_case result JSON: ground track and time history")
    parser.add_argument("--flight-label", default="Square mission")
    parser.add_argument("--flight-name", default="fig1_flight", help="file name of that figure")
    parser.add_argument("--compare", help="a second result JSON, drawn on the ground track")
    parser.add_argument("--compare-label", default="Another run")
    parser.add_argument("--oscillation", nargs=3, metavar=("CLEAN", "BAD", "LEG"),
                        help="two result JSONs: roll and sideways speed on this leg")
    parser.add_argument("--braking", nargs="+", action="append", default=[], metavar="LABEL JSON",
                        help="a label and the result JSONs of one version of the profile; give it "
                             "once per version, the current one last")
    parser.add_argument("--wind", nargs=2, action="append", default=[], metavar=("JSON", "LABEL"),
                        help="a wind study's records and its label; may be given twice")
    parser.add_argument("--oat", help="the one-at-a-time study's records")
    parser.add_argument("--speeds", help="the scenario study's records: each cruise speed, both modes")
    parser.add_argument("--silence", nargs=4, metavar=("JSON", "LABEL", "JSON", "LABEL"),
                        help="two runs with a silence event, the current way of sending last")
    args = parser.parse_args()
    CFG = config.load(args.mission)
    FIGURES = config.REPO / "missions" / args.mission / "figures" / args.model_revision

    vehicle_plan()
    wing_share_curve()
    route_plan()
    if args.flight:
        flight(load(args.flight), args.flight_label,
               load(args.compare) if args.compare else None, args.compare_label, args.flight_name)
    if args.oscillation:
        clean, bad, leg = args.oscillation
        oscillation(load(clean), load(bad), int(leg))
    if args.braking:
        if any(len(group) < 2 for group in args.braking):
            parser.error("--braking takes a label and at least one result JSON")
        braking([(label, [load(path) for path in paths]) for label, *paths in args.braking])
    if args.wind:
        sets = [(label, load(path)) for path, label in args.wind]
        wind_envelope(sets)
        pusher_thrust(sets)
    if args.oat:
        margins(load(args.oat))
    if args.speeds:
        speeds(load(args.speeds))
    if args.silence:
        silence([(args.silence[1], load(args.silence[0])), (args.silence[3], load(args.silence[2]))])


if __name__ == "__main__":
    main()
