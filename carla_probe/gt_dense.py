"""Ground-truth dense perception: a depth and an instance-segmentation camera on every RGB mount.

Used on the `dense` track (sensors view + these arrays) and optionally on `privileged`. It is NOT
Bench2Drive-eligible: CARLA renders the answer directly. The question it answers is how well the
driver does when pixel-level perception is perfect -- an upper bound for what a perception model
(carla_probe/perception_server.py) could ever hand it.

The cameras are spawned by the agent against the world, attached to the hero, rather than declared
in `sensors()`: the leaderboard's sensor validation rejects these types on the SENSORS track, which
is exactly right, and this keeps the declared rig identical across tracks.

Each clone copies its RGB camera's mount and field of view; its resolution is the RGB size times
`scale`, so the intrinsics scale with it and `cameras.json` (written next to the frames) says what
they are. The sim is paused while the driver reads, so a percept waits for the clone's image of the
CURRENT frame before writing it -- never a stale one.

Written per camera, per percept:
    <cam>_depth.npy   float16 metres along the optical axis (z-depth, not ray length), HxW
    <cam>_inst.png    lossless 3-channel uint8: [0] = CARLA semantic tag, [1]*256 + [2] = instance id
                      (stored R,G,B in that order, i.e. read it with cv2 and flip channels)
"""
from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

import carla
import cv2

# CARLA 0.9.15 semantic tags (LibCarla/source/carla/image/CityScapesPalette.h). Written into
# cameras.json so a script does not have to know them.
SEMANTIC_TAGS = {
    0: "unlabeled", 1: "road", 2: "sidewalk", 3: "building", 4: "wall", 5: "fence", 6: "pole",
    7: "traffic_light", 8: "traffic_sign", 9: "vegetation", 10: "terrain", 11: "sky", 12: "pedestrian",
    13: "rider", 14: "car", 15: "truck", 16: "bus", 17: "train", 18: "motorcycle", 19: "bicycle",
    20: "static", 21: "dynamic", 22: "other", 23: "water", 24: "road_line", 25: "ground", 26: "bridge",
    27: "rail_track", 28: "guard_rail",
}


def decode_depth(bgra: np.ndarray) -> np.ndarray:
    """CARLA depth camera BGRA -> metres. depth = (R + G*256 + B*65536) / (2^24 - 1) * 1000."""
    b, g, r = bgra[..., 0].astype(np.float64), bgra[..., 1].astype(np.float64), bgra[..., 2].astype(np.float64)
    return ((r + g * 256.0 + b * 65536.0) / (256.0 ** 3 - 1.0) * 1000.0).astype(np.float32)


def decode_instance(bgra: np.ndarray) -> np.ndarray:
    """CARLA instance-segmentation BGRA -> HxWx3 uint8 [tag, id_hi, id_lo]."""
    return np.stack([bgra[..., 2], bgra[..., 1], bgra[..., 0]], axis=-1)


class DenseRig:
    """Depth + instance-seg clones of the RGB cameras, attached to the hero."""

    KINDS = ("depth", "instance_segmentation")

    def __init__(self, world, hero, cameras: List[Dict[str, Any]], scale: float) -> None:
        self._lock = threading.Lock()
        self._latest: Dict[str, Any] = {}
        self._actors = []
        lib = world.get_blueprint_library()
        for cam in cameras:
            w, h = int(round(cam["width"] * scale)), int(round(cam["height"] * scale))
            for kind in self.KINDS:
                bp = lib.find(f"sensor.camera.{kind}")
                bp.set_attribute("image_size_x", str(w))
                bp.set_attribute("image_size_y", str(h))
                bp.set_attribute("fov", str(cam["fov"]))
                tf = carla.Transform(carla.Location(x=cam["x"], y=cam["y"], z=cam["z"]),
                                     carla.Rotation(roll=cam["roll"], pitch=cam["pitch"], yaw=cam["yaw"]))
                actor = world.spawn_actor(bp, tf, attach_to=hero)
                key = f"{cam['id']}:{kind}"
                actor.listen(lambda img, key=key: self._store(key, img))
                self._actors.append(actor)
        self.keys = [f"{c['id']}:{k}" for c in cameras for k in self.KINDS]

    def _store(self, key: str, img) -> None:
        arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4).copy()
        with self._lock:
            self._latest[key] = (int(img.frame), arr)

    def wait_frame(self, frame: int, timeout_s: float = 10.0) -> bool:
        """True once every clone has delivered an image of `frame` or later."""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self._lock:
                if all(k in self._latest and self._latest[k][0] >= frame for k in self.keys):
                    return True
            time.sleep(0.005)
        return False

    def write(self, out_dir: Path, cams: List[str], frame: int) -> List[str]:
        if not self.wait_frame(frame):
            raise RuntimeError(f"gt_dense: clone images for frame {frame} did not arrive")
        files = []
        with self._lock:
            latest = dict(self._latest)
        for cam in cams:
            f_depth, depth = latest[f"{cam}:depth"]
            f_inst, inst = latest[f"{cam}:instance_segmentation"]
            np.save(out_dir / f"{cam}_depth.npy", decode_depth(depth).astype(np.float16))
            # cv2 writes BGR: store [tag, hi, lo] as R,G,B so a reader that flips gets them in order
            cv2.imwrite(str(out_dir / f"{cam}_inst.png"), decode_instance(inst)[..., ::-1])
            files += [f"{cam}_depth.npy", f"{cam}_inst.png"]
        return files

    def destroy(self) -> None:
        for a in self._actors:
            try:
                if a.is_alive:
                    a.stop()
                    a.destroy()
            except RuntimeError:
                pass
        self._actors = []


def camera_model(cameras: List[Dict[str, Any]], rgb_scale: float, dense_scale: float,
                 full_res: bool) -> Dict[str, Any]:
    """What a script needs to turn pixels into metres: per camera, the mount and the pinhole
    intrinsics of every image size a percept writes. Bench2Drive-eligible on every track -- it is
    the agent's own declared sensor rig."""
    out: Dict[str, Any] = {
        "frame_convention": "CARLA vehicle frame: x forward, y RIGHT, z up (m), yaw + = to the right. "
                            "The probe's ego frame is x forward, y LEFT: left = -y.",
        "pinhole": "fx = fy = (W/2) / tan(fov/2), cx = W/2, cy = H/2. Pixel (u, v) at depth z (metres along "
                   "the optical axis) is camera-frame point X_right = (u-cx)*z/fx, Y_down = (v-cy)*z/fy, Z_fwd = z.",
        "semantic_tags": SEMANTIC_TAGS,
        "cameras": {},
    }
    for c in cameras:
        sizes = {"jpg": rgb_scale}
        if full_res:
            sizes["full_jpg"] = 1.0
        if dense_scale:
            sizes["depth_npy"] = dense_scale
            sizes["inst_png"] = dense_scale
        intr = {}
        for name, s in sizes.items():
            w, h = int(round(c["width"] * s)), int(round(c["height"] * s))
            fx = (w / 2.0) / np.tan(np.radians(float(c["fov"])) / 2.0)
            intr[name] = {"width": w, "height": h, "fx": round(float(fx), 3), "fy": round(float(fx), 3),
                          "cx": w / 2.0, "cy": h / 2.0}
        out["cameras"][c["id"]] = {
            "mount": {k: c[k] for k in ("x", "y", "z", "roll", "pitch", "yaw")},
            "fov_deg": c["fov"],
            "intrinsics": intr,
        }
    return out
