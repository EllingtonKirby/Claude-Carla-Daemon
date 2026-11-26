# Perception library (camera tracks)

Ready-made versions of the tools drivers kept writing for themselves: camera geometry, object
footprints from the perception service, lane markings with a lane tracker, and a zoom. Use any of it,
change it, or ignore it. Everything runs inside the code tool:

```python
import sys
from lib import camera, objects, lanes, zoom      # your script lives next to lib/
```

Do not name your own file `lib.py`: the `lib/` package would shadow it.

`frame_dir` below is the frames directory a percept returned (it holds `<CAM>_full.jpg`, and
`../cameras.json`). Ego frame everywhere: `fwd` metres ahead, `left` metres to your left, origin on
the road under the centre of your car.

## camera: pixels and the road

```python
cams = camera.load(frame_dir)              # {"CAM_FRONT": Camera, ...}, for <CAM>_full.jpg
c = cams["CAM_FRONT"]
c.ground_point(u, v)                       # (fwd, left) where pixel (u, v) meets a flat road, or None
c.to_pixel(fwd, left, up=0.0)              # (u, v) of an ego point, or None if out of view
c.point_at_depth(u, v, z)                  # ego (fwd, left, up) for a depth z along the optical axis
```

Flat-road geometry: exact on a flat road, increasingly wrong with distance on slopes and crests.

## objects: road users as footprints (needs the perception service)

```python
objs = objects.scan(frame_dir)                                  # 4 cameras, ~3 s
objs = objects.scan(frame_dir, cams=["CAM_FRONT"], prompts=["car", "traffic cone"])
print(objects.table(objs))
```

Each object: `label`, `score`, `cam`, `near_fwd`/`near_left` (the point of it nearest to you),
`fwd_min..fwd_max`, `left_min..left_max` (its extent; the side away from you is not visible, so
those bounds are lower bounds), `depth_fwd`/`depth_left` (Depth Pro 2's median point over the mask,
an independent estimate), `agree_m` (how far apart the two estimates are), `truncated` (cut off by
the image edge: the footprint is partial). Your own bonnet and boot are removed.

The footprint takes each mask column's lowest pixel and intersects it with the road, raised 0.2 m for
vehicles (their lowest visible edge is a bumper or sill) and at road level for pedestrians, bicycles
and cones.

Measured offline against CARLA ground truth (223 scenes, road users within 30 m):

| | found | nearest-point error (median) | sideways error (median) |
|---|---|---|---|
| this footprint, on perfect (ground-truth) vehicle masks | 82% | 1.0 m | 0.40 m |
| this footprint, on SAM 3 masks (all road users) | 77% | 0.97 m | 0.47 m |
| for comparison: Depth Pro 2 median point over the mask (to the centre) | | 1.85 m | |

Errors are largest for objects partly hidden behind others, at the image edges, and beyond ~20 m.
When `agree_m` is large or `truncated` is set, look (`zoom`).

## lanes: markings and a tracker

```python
m = lanes.markings(frame_dir)                  # CAM_FRONT; cam="CAM_BACK" looks behind
print(lanes.describe(m))
m["lines"]        # each: left0 (m at fwd 0), left_at_10, slope (rad), curvature, colour, coverage
m["centre_left"]  # middle of the lane you are in (+ = to your left), None if not found
m["width"], m["heading_deg"]                   # heading: + = the lane turns left relative to you
```

It resamples the camera onto the road (a bird's-eye view), finds paint, fits a curve to each marking
over fwd 4-20 m, and picks the nearest pair of markings 2.4-4.6 m apart around you as your lane.

Measured against CARLA ground truth (dense scenes): when it reports a lane, the centre is within
0.15 m (median) and the width within 0.15 m; it reports one in about 60% of frames. It misses
faint or worn paint, fog, junctions, and roads marked on one side only (`centre_left` is then None,
but `lines` still lists what it saw).

```python
t = lanes.LaneTracker()                        # state kept in .lane_tracker.json in your code dir
est = t.update(m, step=step, speed_mps=speed, yaw_deg=compass_deg)   # yaw optional
est  # {"centre_left", "heading_deg", "std_m", "width", "status"}
t.centre_left_at([5, 10, 15])                  # tracked centre at those distances ahead
t.reset()                                      # after you change lanes or pass a junction on purpose
```

A single frame's estimate moves by tens of centimetres from frame to frame. The tracker is a small
Kalman filter on the centre offset and heading, advanced by the distance you drove: a single
disagreeing frame is rejected as an outlier, two in a row re-centre it (a lane change or a real
shift), and after 12 m without a usable frame it starts again. `std_m` is its own uncertainty; it
grows while it is coasting without measurements. Measured on scenes 2 s apart: it halves the
frame-to-frame change (7 to 4 cm) and cuts the worst errors (90th percentile 1.4 to 1.1 m); when no
lane is visible its coasted estimate is poor (about 1 m off), so trust `std_m`.

## zoom: look closer

```python
p = zoom.crop(frame_dir, "CAM_FRONT", (x0, y0, x1, y1), scale=4, brighten=2.5)   # pixels of _full.jpg
p = zoom.around(frame_dir, "CAM_FRONT", fwd=25, left=-3.5, size_m=6)             # box on the road
```

Returns a PNG path in your code dir; read it. Upscaling adds no detail; `brighten` helps in fog.

## scene: TransFuser v6 perception (only in sessions that run it)

```python
s = scene.read(frame_dir)                      # ~1.5 s, all six TFV6 cameras at once
print(scene.describe(s))
s["objects"]      # nearest first: class, fwd/left (centre), near_fwd/near_left (nearest point),
                  # yaw_rel_deg (+ = turned left, ~180 = oncoming), length_m, width_m,
                  # speed_mps (rough), braking, score
s["lanes"]        # same format as lanes.markings(): lines, centre_left, width, heading_deg
s["lights"]       # traffic-light cells ahead: [{"state": "red"|"green", "fwd", "left"}]
scene.is_road(s, fwd, left)                   # True where the map says road or lane marking
png = scene.render(s)                         # bird's-eye picture: forward up, you at the bottom
```

TransFuser v6 is a driving network trained on CARLA; here only its perception is used: boxes for
vehicles, walkers, bikers, parked vehicles (`parking`), emergency vehicles (`special`), obstacles
(cones, barriers), traffic lights and stop signs, and a bird's-eye map 32 m behind to 64 m ahead,
40 m each side, at 0.25 m. It looks through its own six cameras (TFV6_1 ... TFV6_6), which are also
yours to view; the `lanes.LaneTracker` works on `s["lanes"]` as it does on `lanes.markings()`.

Measured offline against CARLA ground truth (30 scenes on two routes, road users within 30 m): 89%
found, nearest-point error 0.29 m (median), sideways 0.15 m. Lanes: found in every frame that had a
clear ground-truth lane (7), centre within 0.31 m. Speeds are rough (one car doing 3.3 m/s read as
7.3), and small or distant pedestrians can be missed.
