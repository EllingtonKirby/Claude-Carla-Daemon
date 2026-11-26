"""Lane markings from a camera, and a tracker that filters the lane estimate over time.

    from lib import lanes
    m = lanes.markings(frame_dir)                 # CAM_FRONT by default
    print(lanes.describe(m))
    t = lanes.LaneTracker()                       # state persists in the code dir between script runs
    est = t.update(m, step=status_step, speed_mps=speed)
    est["centre_left"], est["heading_deg"], est["std_m"]
    t.centre_left_at([5, 10, 15])                 # where the tracked lane centre is, ego frame

markings(): resamples the camera onto a flat-road grid in front of the car (a bird's-eye view, IPM),
finds paint (bright or yellow stripes narrower than ~0.4 m) and fits one straight line per marking
over fwd 4-20 m. Each line: left offset at fwd 0 (m), slope (rad, + = bends to your left), colour
(white / yellow), coverage (share of the range with paint: dashed lines are low, solid high).
The lane you are in is bounded by the nearest line on each side: centre_left is its middle (+ = the
centre is to your left), width its width, heading_deg the mean slope (+ = the lane turns left
relative to your heading). Assumes a flat road; curves are fitted as straight lines, so on a bend
trust the near range. In fog or at night paint fades: n_lines and coverage say how much was seen.

A single frame's estimate moves by tens of centimetres from one frame to the next even on a straight
road. LaneTracker keeps a filtered estimate (a small Kalman filter on centre offset and heading,
advanced by the distance you drove) so one noisy frame does not move it much, and says how sure it
is (std_m). Measurements far from the estimate are rejected as outliers unless they persist (two in a
row about one lane width away: a lane change, and the tracker re-centres on the new lane).
"""
from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

from . import camera as _camera

FWD = np.arange(4.0, 20.01, 0.1)
LEFT = np.arange(-8.0, 8.001, 0.05)


def _bev(frame_dir: str, cam_name: str):
    cam = _camera.load(frame_dir)[cam_name]
    img = cv2.imread(cam.image_path(frame_dir))
    if img is None:
        raise FileNotFoundError(cam.image_path(frame_dir))
    F, L = np.meshgrid(FWD, LEFT, indexing="ij")
    if cam_name == "CAM_BACK":
        F = -F
    pts = np.stack([F, L, np.zeros_like(F)], axis=-1)
    uvz = cam.to_pixels(pts)
    mapx = np.where(uvz[..., 2] > 0.1, uvz[..., 0], -1).astype(np.float32)
    mapy = np.where(uvz[..., 2] > 0.1, uvz[..., 1], -1).astype(np.float32)
    bev = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    valid = (mapx >= 0) & (mapx < img.shape[1]) & (mapy >= 0) & (mapy < img.shape[0])
    return bev, valid


def _paint(bev: np.ndarray, valid: np.ndarray):
    hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
    # road surface: the colour of the patch right in front of the car (fwd 4-6 m, |left| < 1 m), and
    # anything close to it; paint only counts next to road (verges, kerbs and fields do not)
    fi = (lanes_fwd_idx(4.0), lanes_fwd_idx(6.0)); li = (lanes_left_idx(-1.0), lanes_left_idx(1.0))
    patch = hsv[fi[0]:fi[1], li[0]:li[1]].reshape(-1, 3).astype(float)
    pv = valid[fi[0]:fi[1], li[0]:li[1]].reshape(-1)
    if pv.sum() < 20:
        road = valid.copy()
    else:
        med = np.median(patch[pv], axis=0)
        road = (np.abs(hsv[..., 2].astype(float) - med[2]) < 35) & (hsv[..., 1] < max(60, med[1] + 30)) & valid
        road = cv2.morphologyEx(road.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)
    near_road = cv2.dilate(road.astype(np.uint8), np.ones((1, 13), np.uint8)).astype(bool)   # +-0.3 m sideways
    eq = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    # paint is brighter than the road on both sides, across a width < ~0.4 m (8 cells of 5 cm)
    tophat = cv2.morphologyEx(eq, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 1)))
    cand = valid & (hsv[..., 1] < 70)                      # white paint is unsaturated
    thr = max(15.0, float(np.percentile(tophat[valid], 97)) * 0.5) if valid.any() else 255
    white = (tophat > thr) & cand
    yellow = (hsv[..., 0] >= 15) & (hsv[..., 0] <= 35) & (hsv[..., 1] > 90) & (hsv[..., 2] > 100) & valid & near_road
    return white | yellow, yellow


def lanes_fwd_idx(f: float) -> int:
    return int(np.clip(np.searchsorted(FWD, f), 0, len(FWD) - 1))


def lanes_left_idx(l: float) -> int:
    return int(np.clip(np.searchsorted(LEFT, l), 0, len(LEFT) - 1))


def markings(frame_dir: str, cam: str = "CAM_FRONT") -> Dict:
    """Lane markings seen by one camera (CAM_FRONT, or CAM_BACK looking behind; offsets are always
    in your ego frame, fwd measured along the camera's view)."""
    bev, valid = _bev(frame_dir, cam)
    paint, yellow = _paint(bev, valid)
    return lines_from_paint(paint, yellow, cam)


def lines_from_paint(paint: np.ndarray, yellow: np.ndarray, cam: str = "CAM_FRONT") -> Dict:
    """Fit marking lines to a bird's-eye paint mask on the FWD x LEFT grid (used by markings())."""
    prof = paint.sum(axis=0).astype(float)                  # per left column, over all fwd
    prof = np.convolve(prof, np.ones(5) / 5, mode="same")
    lines = []
    order = np.argsort(-prof)
    taken = np.zeros_like(prof, bool)
    for j in order:
        if prof[j] < 0.06 * len(FWD):
            break
        if taken[j]:
            continue
        lo, hi = max(0, j - 14), min(len(LEFT), j + 15)    # +-0.7 m: one marking
        taken[lo:hi] = True
        band = paint[:, lo:hi]
        rows, cols = np.nonzero(band)
        if rows.size < 15:
            continue
        f = FWD[rows]; l = LEFT[lo + cols]
        # robust fit left = a + b * fwd + c * fwd^2 (c = 0 when the paint is too short to tell)
        deg = 2 if (f.max() - f.min()) > 8 else 1
        co = np.polyfit(f, l, deg)
        for _ in range(3):
            r = l - np.polyval(co, f)
            keep = np.abs(r) < max(0.10, 2.5 * np.median(np.abs(r)))
            if keep.sum() < 10:
                break
            co = np.polyfit(f[keep], l[keep], deg)
        c2 = float(co[0]) if deg == 2 else 0.0
        b, a = float(co[-2]), float(co[-1])
        cover = float(np.mean(band.any(axis=1)))
        is_yellow = float(yellow[:, lo:hi][band].mean()) > 0.5 if band.any() else False
        lines.append({"left0": round(float(a), 2), "slope": round(float(b), 4), "colour": "yellow" if is_yellow else "white",
                      "coverage": round(cover, 2), "left_at_10": round(float(a + 10 * b + 100 * c2), 2),
                      "curvature": round(2 * c2, 4)})
    lines = [d for d in lines if d["coverage"] >= 0.15]
    # a thick marking can yield two fits half a metre apart: keep the better-covered one
    kept = []
    for d in sorted(lines, key=lambda d: -d["coverage"]):
        if all(abs(d["left_at_10"] - k["left_at_10"]) > 0.5 for k in kept):
            kept.append(d)
    lines = kept
    lines.sort(key=lambda d: d["left_at_10"])
    if cam == "CAM_BACK":                                   # fitted with fwd negated: report in ego terms
        for d in lines:
            d["slope"] = -d["slope"]
    out = {"cam": cam, "lines": lines, "n_lines": len(lines), "centre_left": None, "width": None, "heading_deg": None}
    # the lane you are in: the pair of lines, one on each side, 2.4-4.6 m apart, roughly parallel,
    # that is nearest to you; ties go to the better-painted pair
    best = None
    for R0 in (d for d in lines if d["left0"] < 0.2):
        for L0 in (d for d in lines if d["left0"] > -0.2):
            w = L0["left0"] - R0["left0"]
            if not 2.4 <= w <= 4.6 or abs(L0["slope"] - R0["slope"]) > 0.06:
                continue
            centre = (L0["left0"] + R0["left0"]) / 2
            if abs(centre) > w / 2 + 0.3:
                continue
            cost = abs(centre) - 0.5 * (L0["coverage"] + R0["coverage"])
            if best is None or cost < best[0]:
                best = (cost, R0, L0)
    if best:
        _, R0, L0 = best
        out["centre_left"] = round((L0["left0"] + R0["left0"]) / 2, 2)
        out["width"] = round(L0["left0"] - R0["left0"], 2)
        out["heading_deg"] = round(math.degrees((L0["slope"] + R0["slope"]) / 2), 2)
        out["bounds"] = [R0, L0]
    return out


def describe(m: Dict) -> str:
    s = [f"{m['cam']}: {m['n_lines']} markings"]
    for d in m["lines"]:
        s.append(f"  {d['colour']:6} left {d['left0']:+5.2f} m at fwd 0, {d['left_at_10']:+5.2f} at 10 m, "
                 f"slope {math.degrees(d['slope']):+4.1f} deg, coverage {d['coverage']:.2f}")
    if m.get("centre_left") is not None:
        s.append(f"  lane centre {m['centre_left']:+.2f} m (+ = left of you), width {m['width']:.2f} m, "
                 f"heading {m['heading_deg']:+.1f} deg")
    return "\n".join(s)


class LaneTracker:
    """Filtered lane-centre offset and heading, persisted in a JSON file between script runs.

    update(m, step, speed_mps, yaw_deg=None): m from markings(); step from the session status;
    speed_mps from status; yaw_deg (optional) any heading reading that is consistent from call to call
    (e.g. a compass), which lets the tracker account for your own turning between frames.
    reset(): forget the lane (after you change lanes on purpose, or after a junction).
    """

    def __init__(self, path: Optional[str] = None, meas_std: float = 0.25, heading_std_deg: float = 2.5) -> None:
        d = os.environ.get("CARLA_PROBE_CODE_DIR") or os.getcwd()
        self.path = path or os.path.join(d, ".lane_tracker.json")
        self.R = np.diag([meas_std ** 2, math.radians(heading_std_deg) ** 2])
        try:
            with open(self.path) as f:
                self.s = json.load(f)
        except Exception:
            self.s = {}

    def _save(self) -> None:
        with open(self.path, "w") as f:
            json.dump(self.s, f)

    def reset(self) -> None:
        self.s = {}
        self._save()

    def update(self, m: Dict, step: int, speed_mps: float, yaw_deg: Optional[float] = None) -> Dict:
        z = None
        if m.get("centre_left") is not None and m.get("heading_deg") is not None:
            z = np.array([m["centre_left"], math.radians(m["heading_deg"])])
        s = self.s
        if not s or step - s.get("step", -10 ** 9) > 200 or step < s.get("step", 0):
            if z is None:
                return {"centre_left": None, "heading_deg": None, "std_m": None, "status": "no lane seen yet"}
            self.s = {"x": z.tolist(), "P": (self.R * 2).tolist(), "step": step, "yaw": yaw_deg, "pending": None,
                      "width": m.get("width")}
            self._save()
            return self._report("initialised")
        x = np.array(s["x"]); P = np.array(s["P"])
        dist = max(0.0, speed_mps) * (step - s["step"]) * 0.05
        s["since_m"] = s.get("since_m", 0.0) + dist
        if s["since_m"] > 12.0 and z is not None:      # coasted too far on a guess: start again
            self.s = {"x": z.tolist(), "P": (self.R * 2).tolist(), "step": step, "yaw": yaw_deg, "pending": None,
                      "width": m.get("width"), "since_m": 0.0}
            self._save()
            return self._report("re-initialised after 12 m without a usable measurement")
        # predict: driving dist with the lane heading h (relative to yours) moves the centre's left
        # offset by dist * tan(h); your own turning to the right (yaw) turns the lane left by as much.
        dyaw = 0.0
        if yaw_deg is not None and s.get("yaw") is not None:
            dyaw = math.radians(((yaw_deg - s["yaw"]) + 180) % 360 - 180)   # + = you turned right
        x = np.array([x[0] + dist * math.tan(x[1]), x[1] + dyaw])
        q_off = (0.05 + 0.02 * dist) ** 2 + (0 if yaw_deg is not None else (0.01 * dist) ** 2)
        q_h = math.radians(0.5 + (0 if yaw_deg is not None else 1.5)) ** 2
        P = P + np.diag([q_off, q_h])
        status = "predicted (no lane measurement)"
        if z is not None:
            S = P + self.R
            innov = z - x
            maha = float(innov @ np.linalg.solve(S, innov))
            width = s.get("width") or m.get("width") or 3.5
            if maha > 9.0:
                # one disagreeing frame is treated as an outlier; two in a row that agree with each other
                # mean the lane really moved (a lane change, a junction, a drift the filter missed)
                pend = s.get("pending")
                if pend is not None and abs(z[0] - pend) < 0.6:
                    x, P = z, self.R * 2
                    status = ("lane change: re-centred on the new lane" if abs(abs(innov[0]) - width) < 1.0
                              else "re-initialised: two measurements in a row disagreed with the estimate")
                    s["pending"] = None
                else:
                    s["pending"] = float(z[0]); status = f"measurement rejected as an outlier ({innov[0]:+.2f} m)"
            else:
                K = P @ np.linalg.inv(S)
                x = x + K @ innov
                P = (np.eye(2) - K) @ P
                s["pending"] = None
                s["since_m"] = 0.0
                status = "updated"
                if m.get("width"):
                    s["width"] = 0.8 * (s.get("width") or m["width"]) + 0.2 * m["width"]
        s.update({"x": x.tolist(), "P": P.tolist(), "step": step, "yaw": yaw_deg if yaw_deg is not None else s.get("yaw")})
        self.s = s
        self._save()
        return self._report(status)

    def _report(self, status: str) -> Dict:
        x = self.s["x"]; P = np.array(self.s["P"])
        return {"centre_left": round(x[0], 2), "heading_deg": round(math.degrees(x[1]), 2),
                "std_m": round(math.sqrt(P[0, 0]), 2), "width": round(self.s.get("width") or 0, 2) or None,
                "status": status}

    def centre_left_at(self, fwds: Sequence[float]) -> List[float]:
        """Left offset of the tracked lane centre at each fwd distance (straight-lane model)."""
        if not self.s:
            return []
        c, h = self.s["x"]
        return [round(c + f * math.tan(h), 2) for f in fwds]
