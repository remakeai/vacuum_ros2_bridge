#!/usr/bin/env python3
"""Analyze an in-place-spin rosbag (mcap) to find:
  1. LiDAR fixed-pattern-noise (FPN) bins -- a near-constant return at a fixed
     angle/distance regardless of robot rotation (e.g. enclosure posts).
  2. The LiDAR optical-center offset (x,y) from the robot's rotation center, by
     finding the offset that makes the spinning scan sharpest (points from all
     yaws land on the same world cells). A wrong offset makes stationary points
     wobble radially during rotation.

Decodes the bag on the host via mcap (no ROS install needed).
"""
import math
import sys
import numpy as np
from mcap_ros2.reader import read_ros2_messages

BAG = sys.argv[1] if len(sys.argv) > 1 else r"C:\maps\bag\proscenic_0.mcap"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


# ---------------------------------------------------------------- read the bag
scans = []          # (t, ranges[np], angle_min, angle_inc, range_max)
odom = []           # (t, x, y, yaw)
static_tf = {}      # child -> (parent, x, y, yaw)

print("Reading bag (this takes ~30-60s)...")
for m in read_ros2_messages(BAG, topics=["/scan", "/odom", "/tf_static"]):
    topic = m.channel.topic
    msg = m.ros_msg
    if topic == "/scan":
        r = np.array(msg.ranges, dtype=np.float64)
        scans.append((stamp_s(msg.header), r, msg.angle_min,
                      msg.angle_increment, msg.range_max))
    elif topic == "/odom":
        p = msg.pose.pose
        odom.append((stamp_s(msg.header), p.position.x, p.position.y,
                     yaw_of(p.orientation)))
    elif topic == "/tf_static":
        for tr in msg.transforms:
            t = tr.transform.translation
            static_tf[tr.child_frame_id] = (
                tr.header.frame_id, t.x, t.y, yaw_of(tr.transform.rotation))

print(f"  {len(scans)} scans, {len(odom)} odom samples, "
      f"{len(static_tf)} static transforms")
if not scans or not odom:
    sys.exit("Need both /scan and /odom in the bag.")

nb = len(scans[0][1])
angle_min = scans[0][2]
angle_inc = scans[0][3]
range_max = scans[0][4]
bearings = angle_min + np.arange(nb) * angle_inc

# odom interpolators (unwrap yaw)
odom.sort()
ot = np.array([o[0] for o in odom])
ox_ = np.array([o[1] for o in odom])
oy_ = np.array([o[2] for o in odom])
oyaw = np.unwrap(np.array([o[3] for o in odom]))


def odom_at(t):
    return (np.interp(t, ot, ox_), np.interp(t, ot, oy_), np.interp(t, ot, oyaw))


# Disconnects leave gaps where odom froze while the robot kept turning (and jumps
# on reconnect). odom yaw is only trustworthy within a continuous segment, so pick
# the segment with the most rotation for the offset fit.
gap_idx = np.where(np.diff(ot) > 0.5)[0]
seg_bounds = np.split(np.arange(len(ot)), gap_idx + 1)
best_seg = max(seg_bounds, key=lambda s: oyaw[s].max() - oyaw[s].min())
seg_t0, seg_t1 = ot[best_seg[0]], ot[best_seg[-1]]
seg_span_deg = math.degrees(oyaw[best_seg].max() - oyaw[best_seg].min())
print(f"odom: {len(gap_idx)} disconnect gaps -> {len(seg_bounds)} segments; "
      f"largest = {seg_span_deg:.0f} deg over {seg_t1 - seg_t0:.0f}s (used for offset fit)")


# ---------------------------------------------------------------- 1. FPN
R = np.full((len(scans), nb), np.nan)
for i, s in enumerate(scans):
    r = s[1].copy()
    r[~np.isfinite(r)] = np.nan
    r[(r <= 0.0) | (r >= range_max)] = np.nan
    R[i] = r

finite = np.isfinite(R)
ret_rate = finite.mean(axis=0)
with np.errstate(invalid="ignore"):
    mean_r = np.nanmean(R, axis=0)
    std_r = np.nanstd(R, axis=0)

# total robot rotation in the bag -> a real point at a fixed bearing should sweep
# a large range of distances; FPN stays put. Flag high return-rate + low std.
total_rot = abs(oyaw[-1] - oyaw[0])
print(f"\nTotal rotation in bag: {math.degrees(total_rot):.0f} deg")
FPN_STD = 0.07      # m: a real point sweeps far more than this during a spin
FPN_RATE = 0.65
fpn = np.where((ret_rate > FPN_RATE) & (std_r < FPN_STD) & np.isfinite(std_r))[0]

print("\n========== FIXED-PATTERN NOISE ==========")
if len(fpn) == 0:
    print("  none found (no bins with constant return across the spin)")
else:
    # group contiguous bins (handle wrap)
    groups = []
    cur = [fpn[0]]
    for b in fpn[1:]:
        if b == cur[-1] + 1:
            cur.append(b)
        else:
            groups.append(cur); cur = [b]
    groups.append(cur)
    if len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == nb - 1:
        groups[0] = groups[-1] + groups[0]; groups.pop()
    deg = 360.0 / nb
    mask_terms = []
    for g in groups:
        lo, hi = g[0] * deg, g[-1] * deg
        d = np.nanmean(mean_r[g])
        print(f"  bins {g[0]:3d}-{g[-1]:3d}  ({lo:5.1f}-{hi:5.1f} deg)  "
              f"~{d:.3f} m  return {ret_rate[g].mean()*100:.0f}%  std {std_r[g].mean()*1000:.0f}mm")
        pad = deg  # one bin of margin each side
        mask_terms.append(f"{lo - pad:.0f},{hi + pad:.0f}")
    print(f'\n  -> scan_mask_deg:="{",".join(mask_terms)}"')

# ---------------------------------------------------------------- 2. LiDAR offset
# Use scans spread across the whole rotation; drop FPN bins so they don't bias it.
keep = np.ones(nb, dtype=bool)
keep[fpn] = False
in_seg = [i for i, s in enumerate(scans) if seg_t0 <= s[0] <= seg_t1]
N = min(500, len(in_seg))
sel = [in_seg[k] for k in np.linspace(0, len(in_seg) - 1, N).astype(int)]

px_s, py_s, th_s, bx_s, by_s = [], [], [], [], []
for i in sel:
    t, r, *_ = scans[i]
    valid = np.isfinite(R[i]) & keep
    if valid.sum() < 20:
        continue
    rr = R[i][valid]
    b = bearings[valid]
    bxx, byy, yaw = odom_at(t)
    px_s.append(rr * np.cos(b)); py_s.append(rr * np.sin(b))
    th_s.append(np.full(rr.shape, yaw))
    bx_s.append(np.full(rr.shape, bxx)); by_s.append(np.full(rr.shape, byy))

rcos = np.concatenate(px_s); rsin = np.concatenate(py_s)
theta = np.concatenate(th_s); bx = np.concatenate(bx_s); by = np.concatenate(by_s)
ct, st = np.cos(theta), np.sin(theta)
print(f"\nOffset fit over {len(sel)} scans, {rcos.size} points")

# fixed grid edges from the (0,0)-offset extent
ox0 = ct * rcos - st * rsin + bx
oy0 = st * rcos + ct * rsin + by
pad = 0.5
xe = np.arange(ox0.min() - pad, ox0.max() + pad, 0.03)
ye = np.arange(oy0.min() - pad, oy0.max() + pad, 0.03)


def sharpness(offx, offy):
    pxx = offx + rcos
    pyy = offy + rsin
    ux = ct * pxx - st * pyy + bx
    uy = st * pxx + ct * pyy + by
    H, _, _ = np.histogram2d(ux, uy, bins=[xe, ye])
    return float(np.sum(H * H))


def search(cx, cy, half, step):
    best, bxy = -1, (cx, cy)
    rng = np.arange(-half, half + 1e-9, step)
    for dx in rng:
        for dy in rng:
            s = sharpness(cx + dx, cy + dy)
            if s > best:
                best, bxy = s, (cx + dx, cy + dy)
    return bxy, best


(bx0, by0), s_coarse = search(0.0, 0.0, 0.14, 0.02)
(bestx, besty), s_fine = search(bx0, by0, 0.02, 0.004)

base_sharp = sharpness(0.0, 0.0)
print("\n========== LiDAR OFFSET (base_footprint -> base_scan) ==========")
print(f"  sharpness  @ (0,0):        {base_sharp:.0f}")
print(f"  sharpness  @ best:         {s_fine:.0f}  ({s_fine/base_sharp:.2f}x sharper)")
print(f"  BEST OFFSET:  x = {bestx:+.3f} m   y = {besty:+.3f} m")

# current value from tf_static, composing the chain up to base_footprint
def compose_to(child, target="base_footprint"):
    x, y, yaw = 0.0, 0.0, 0.0
    node = child
    for _ in range(10):
        if node == target or node not in static_tf:
            break
        parent, px, py, pyaw = static_tf[node]
        c, s = math.cos(pyaw), math.sin(pyaw)
        x, y = px + c * x - s * y, py + s * x + c * y
        yaw += pyaw
        node = parent
    return x, y, node

if "base_scan" in static_tf:
    cx, cy, root = compose_to("base_scan")
    print(f"  current URDF (from tf_static, chain to {root}):  "
          f"x = {cx:+.3f} m   y = {cy:+.3f} m")
    print(f"  correction needed:  dx = {bestx-cx:+.3f} m   dy = {besty-cy:+.3f} m")
print("=================================================================")
