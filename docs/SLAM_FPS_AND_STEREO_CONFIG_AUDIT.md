# Two config suspicions, checked against the ORB-SLAM3 source — 2026-10-14

Both were raised while looking for the cause of the tracking collapse behind
the reference-arm results (41–202 `Tracking LOST` and 2–14 `New Map created`
per run, one loop detection across three complete orbits). Neither should
have been acted on without reading the code, and reading it separated them:
one is a non-issue, the other is real and leads somewhere further.

Source audited is the tree the container actually builds. `Dockerfile` has
two stages — `nvidia_gpu` copies `FastTrack` (line 101), `cpu` copies
`ORB_SLAM3` (line 128) — and `ARG TARGET=cpu` with `FROM ${TARGET} AS final`
makes `cpu` the default, so `ORB_SLAM3/` is the live tree. `File.version:
"1.0"` in our config routes `System.cc:77-79` to the `Settings` class rather
than the legacy loader, so the `Settings`-based paths below are the ones
that run.

## 1. `Stereo.T_c1_c2` vs `RIGHT.P` — contradiction is real, effect is nil

`bluerov2_stereo.yaml` puts the baseline on +Y:

    Stereo.T_c1_c2: [1,0,0,0;  0,1,0,0.145;  0,0,1,0;  0,0,0,1]

while `RIGHT.P[0,3] = -60.47 = -fx * 0.145` puts it on -X. They disagree
about the axis. Tracing what is actually consumed:

- `Camera.type: "PinHole"` sets `cameraType_ = PinHole` (`Settings.cc:192`).
- `readCamera2` under `PinHole` sets `bNeedToRectify_ = true`
  (`Settings.cc:280`) and builds a `Pinhole` for `calibration2_`.
- Stereo extrinsics (`Settings.cc:337-349`): the `cameraType_ == Rectified`
  branch is **not** taken, so the `else` reads `Stereo.T_c1_c2` — and uses
  **only its norm**: `b_ = Tlr_.translation().norm()` = 0.145. The direction
  is discarded at this point.
- `precomputeRectificationMaps()` then overwrites `bf_ = b_ * P1(0,0)`
  (`Settings.cc:518`) = 0.145 x 417.03 = 60.47, agreeing with `RIGHT.P`.
- `Tlr_` only reaches tracking inside `if (... && settings->cameraType() ==
  Settings::KannalaBrandt)` (`Tracking.cc:561-566`). Ours is `PinHole`, so
  `mpCamera2` stays `nullptr` and `mTlr` is never assigned.
- With `mpCamera2` null, `Tracking.cc:1493` takes the single-camera stereo
  constructor and `Frame::ComputeStereoMatches()` (`Frame.cc:141`), not the
  two-camera `ComputeStereoFishEyeMatches()` that would have used `mTlr`.

**Verdict: no effect on tracking. Do not change this expecting a fix.**

It is still worth correcting as hygiene, because it is a loaded gun. The
two-camera path at `Frame.cc:1060` does
`static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0]` unconditionally —
on a `Pinhole` that is undefined behaviour. The moment anyone switches
`Camera.type` to `KannalaBrandt`, the +Y baseline becomes live and that cast
fires. The 0.145 belongs in element [0,3], not [1,3].

## 2. `Camera.fps: 30` against a measured 10.7–14 fps — real, three effects

`mMaxFrames = settings->fps()` = 30 and `mMinFrames = 0`
(`Tracking.cc:584-585`). `mMaxFrames` is a count of **frames**, and the code
is written assuming frames arrive at the declared rate. At the measured
tracking rate of 10.7–14 fps, every window built from it is **2.1–2.8 s long
where the author intended 1 s**:

| site | what it gates | consequence of the 2.5x stretch |
|---|---|---|
| `Tracking.cc:3159` (`c1a`) | forced keyframe after `mMaxFrames` frames | keyframes forced every ~2.5 s instead of ~1 s; sparser map during fast motion |
| `Tracking.cc:3033` | after a relocalisation, `TrackLocalMap` returns false unless inliers >= **50** | the harsher post-reloc inlier bar is held ~2.5x longer, so recovery is more likely to be declared a failure |
| `Tracking.cc:3094` | after a relocalisation, no keyframe insertion while `nKFs > mMaxFrames` | the map cannot grow for ~2.5 s immediately after recovering, making a second loss more likely |

The last two sit directly in the loss -> relocalise -> loss cycle, which is
where 41–202 failures per run are occurring. This is a plausible contributor,
not a demonstrated cause.

## 3. What the fps check actually uncovered: 60 % of frames never arrive

`bluerov2_turbine.scn` includes `bluerov2.scn`, which declares both stereo
cameras at **`rate="30.0"`**. ORB-SLAM3 reports a tracking rate of 10.7–14
fps. Roughly **three frames in five are being dropped** somewhere between
Stonefish and `Track()`.

That matters more than the declaration does. Dropped frames mean the motion
between consecutive *processed* frames is ~2.5x larger than the system was
tuned for, which degrades feature matching and the constant-velocity motion
model directly — a far more likely driver of tracking loss than any
threshold.

So there are two different fixes and they are not equivalent:

- **Declare the truth.** Set `Camera.fps` to the rate actually achieved
  (~12). One line, reversible, makes the frame-count windows mean what the
  author intended. Does nothing about inter-frame motion.
- **Close the gap.** Find where the other 60 % go. The image-enhancement
  nodes sit between the simulator and SLAM (`/bluerov2/left/image_enhanced`)
  and are Python with `cv_bridge` conversions; if they run at ~12 Hz they
  throttle everything downstream. Harder, but it halves inter-frame motion
  and would make the declaration question moot.

## Next step, and it is a measurement not a change

Before touching either: during the next run, measure where the rate is lost.

    ros2 topic hz /bluerov2/left/image_raw
    ros2 topic hz /bluerov2/left/image_enhanced

If `image_raw` is 30 and `image_enhanced` is 12, the enhancement nodes are
the bottleneck and that is the thing to fix. If both are 30, ORB-SLAM3 itself
is the bottleneck and the declaration fix is the available one. If
`image_raw` is already 12, Stonefish is not meeting its declared rate and
neither fix is right.

The falsifiable test of the fps declaration itself is one run at the same
seed with `Camera.fps` corrected, compared on `Tracking LOST` and
`New Map created` counts from the container console log — the same counters
that produced the table above.

Related: `docs/TASK_15_SLAM_MAP_FRAGMENTATION_WRITEUP.md` reached the
uncorrelated-theta and map-crash observations independently; neither the fps
path nor the frame drop appears there.
