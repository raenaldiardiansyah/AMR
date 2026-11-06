#!/usr/bin/env python3
"""
amr_test_utils.py
-----------------
Shared helpers for the Chapter IV (BAB IV) data-collection scripts.

Common functions: angle normalization (wrapTo180), yaw from quaternion,
position RMSE at discrete points, simple statistics, CSV saving, and
timestamped output paths. Import this from each test script.
"""

import os
import csv
import math
import statistics
from datetime import datetime

# Physical constants (keep in ONE place; update here if calibration changes)
WHEEL_DIAMETER = 0.150      # m  (robot besar, ban 15 cm; robot kecil lama: 0.068)
WHEEL_BASE     = 0.456      # m  (robot besar; robot kecil lama: 0.299)
TICKS_PER_REV  = 2400.0     # 600 PPR x4, 1:1 (robot kecil lama: 4557)
M_PER_TICK     = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV

# All BAB IV data lands under ~/thesis_data/ alongside pengujian_3 / pengujian_4
# so the whole thesis dataset stays in one place (option A, 2026-06-21).
DEFAULT_OUTPUT_DIR = os.path.expanduser('~/thesis_data/pengujian_6_pid')

# ---------------------------------------------------------------- warehouse waypoints
# All coordinates are Nav2 goal positions (map frame), NOT raw tape/REAL
# coordinates -- see per-point provenance below. REAL tape values are in
# measure_map_geometry.py's REAL dict for reference/BAB IV comparison.
#
# 2026-07-07, second pass: after the 2026-07-07 map edit
# (warehouse_v1_edited_edited.yaml) broke several literal tape coordinates
# (NO_VALID_PATH at A2, near-wall-collision at A1 -- see test9.md), two
# rounds of recalibration were attempted:
#   - Round 1 (distance-transform, offline): point of max wall-clearance
#     within 50cm of the tape mark, computed from the static map. Safe but
#     not grounded in a real measurement.
#   - Round 2 (live /amcl_pose, manual park-and-read, THIS is what's used
#     below): operator parked the robot at each REAL tape mark and read
#     /amcl_pose directly via ils_gui. Verified point-by-point against
#     nearest-REAL-neighbor matching to rule out the swap/mislabel problem
#     that invalidated an earlier same-day attempt (see test9.md) -- this
#     round had NO swaps, all points self-consistent. These raw AMCL
#     readings are used AS-IS (not curve-fitted) per operator preference,
#     since they are direct ground-truth measurements of "where AMCL says I
#     am when I'm physically standing at the REAL rack" -- fitting would
#     smooth over real per-point drift, not just noise. All confirmed
#     plannable via compute_path_to_pose except C3.
#   - A2, Stage, Home: operator confirmed these were already correct,
#     kept unchanged (A2 is the Round-1 re-centered value; Stage and Home
#     are the literal REAL/tape values, already safe).
#   - C3: raw AMCL reading (-0.668,-4.585) sits in a wide (~1.2m) zone of
#     near-max costmap cost (flat ~100, not a normal wall-decay gradient --
#     likely a real obstacle/tight clearance at that exact spot when
#     measured), NO_VALID_PATH confirmed via compute_path_to_pose even
#     after searching 80cm around it. Fell back to the Round-1
#     distance-transform value for this one point only.
WAREHOUSE_WAYPOINTS = {
    # 2026-07-24: RECALIBRATED against warehouse_v3_20260721_edited.yaml
    # (new map -- LiDAR raised +12cm + loop-closure loosened, see test13.md).
    # All 14 live AMCL reads, one point at a time with live neighbor cross-
    # check (learned from the 2026-07-07 swap failure). Mean point error
    # 14.3cm vs tape (down from 29.6cm on the original map, see test3.md).
    # Old (pre-2026-07-24) values are superseded -- do not use.
    # --- Start / staging ---
    'Home':       (0.019,  -0.078),
    'Stage':      (3.620,   0.461),
    # --- Rack A (X = 4.0) ---
    'A1':         (3.904, -8.527),
    'A2':         (3.973, -6.904),
    'A3':         (4.104, -4.654),
    'A4':         (4.103, -3.161),
    # --- Rack B (X = 1.5) ---
    'B1':         (1.567, -8.486),
    'B2':         (1.491, -7.124),
    'B3':         (1.595, -4.715),
    'B4':         (1.570, -2.911),
    # --- Rack C (X = -1.0) ---
    'C1':        (-1.072, -8.540),
    'C2':        (-1.044, -7.066),
    'C3':        (-0.970, -4.566),
    'C4':        (-0.985, -2.912),  # re-measured 2026-07-24 after EKF restart (Vyaw reverted), error dropped 29.5cm->8.9cm
    # --- Cross-section row 1 (Y = -5.7) ---
    'X1_A':       (4.0,  -5.7),
    'X1_B':       (1.5,  -5.7),
    'X1_C':      (-1.0,  -5.7),
    # --- Cross-section row 2 (Y = -1.5) ---
    'X2_A':       (3.5,  -1.5),
    'X2_B':       (1.5,  -1.5),
    # --- Cross-section row 3 (Y = 0) ---
    'X3_B':       (1.5,   0.0),
    'X3_C':      (-1.0,   0.0),
    # --- T-junction ---
    'XA_T-junc':  (4.0,  -1.5),
}


# ---------------------------------------------------------------- angles
def load_map_image(map_yaml_path):
    """Load a SLAM map (yaml + image) and return (image_array, extent) ready
    for ax.imshow(img, extent=extent, origin='lower', cmap='gray') in
    map-frame meters."""
    import yaml
    import numpy as np
    from PIL import Image
    with open(os.path.expanduser(map_yaml_path)) as f:
        meta = yaml.safe_load(f)
    map_dir = os.path.dirname(os.path.abspath(os.path.expanduser(map_yaml_path)))
    img = Image.open(os.path.join(map_dir, meta['image']))
    width, height = img.size
    resolution = meta['resolution']
    origin_x, origin_y = meta['origin'][0], meta['origin'][1]
    extent = [origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution]
    return np.array(img), extent


def wrap_to_180(angle_deg):
    """Normalize an angle in DEGREES to (-180, 180]."""
    a = (angle_deg + 180.0) % 360.0 - 180.0
    return a + 360.0 if a <= -180.0 else a


def wrap_to_pi(angle_rad):
    """Normalize an angle in RADIANS to (-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(qx, qy, qz, qw):
    """Yaw (rad) from a quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


# ---------------------------------------------------------------- metrics
def position_error(x_est, y_est, x_gt, y_gt):
    """Euclidean distance between an estimate and ground truth (same units)."""
    return math.hypot(x_est - x_gt, y_est - y_gt)


def rmse(errors):
    """Root-mean-square of a list of scalar errors."""
    errors = [e for e in errors if e is not None and not math.isnan(e)]
    if not errors:
        return float('nan')
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def position_rmse(est_points, gt_points):
    """RMSE of position over matched (x,y) estimate vs ground-truth pairs.
    est_points / gt_points: lists of (x, y)."""
    errs = [position_error(ex, ey, gx, gy)
            for (ex, ey), (gx, gy) in zip(est_points, gt_points)]
    return rmse(errs)


def summarize(values):
    """Return mean/std/min/max/n for a list of numbers (ignores nan)."""
    vals = [v for v in values if v is not None and not math.isnan(v)]
    if not vals:
        return {'n': 0, 'mean': float('nan'), 'std': float('nan'),
                'min': float('nan'), 'max': float('nan')}
    return {
        'n': len(vals),
        'mean': statistics.mean(vals),
        'std': statistics.pstdev(vals) if len(vals) > 1 else 0.0,
        'min': min(vals),
        'max': max(vals),
    }


# ---------------------------------------------------------------- I/O
def timestamped_path(prefix, ext='csv', out_dir=None, label=None):
    """Build output/<prefix>[_<label>]_<YYYYmmdd_HHMMSS>.<ext>."""
    out_dir = out_dir or DEFAULT_OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    name = f"{prefix}_{label}_{ts}" if label else f"{prefix}_{ts}"
    return os.path.join(out_dir, f"{name}.{ext}")


def save_csv(path, fieldnames, rows):
    """Write a list of dict rows to CSV."""
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    return path


def print_summary_table(title, summary_dict, unit=''):
    """Pretty-print a summarize() result."""
    s = summary_dict
    print(f"  {title}: n={s['n']}  mean={s['mean']:.4f}{unit}  "
          f"std={s['std']:.4f}  min={s['min']:.4f}  max={s['max']:.4f}")
