# Pre-registration: why the wrong-face closures fall at the seabed end

Written 2026-10-25, before the run. Follow-up to the manuscript (Section 4.2b), not part of it.

## Question

9 of 12 wrong-face closures fell at 20 m depth or more, at the bottom of the 1 to 22 m face sweep. Two candidate reasons were named and not tested: (A) the image gets darker and poorer in features with depth in the water model, so the recogniser has less to tell faces apart with; (B) at the bottom the view is dominated by the base, the lowest X-brace and the seabed, which are the same on every face, so the views of different faces are more alike there than higher up.

## Run

One stock mission, as the stage-1 flights: `scenario:=bluerov2_turbine dvl_error:=a50 dvl_seed:=1 dr_reference:=true`, fixed RISE, SLAM on, recorded with `IMAGES=1 bash record_run.sh imgL0 1` (left camera, rendered and enhanced, 1 Hz). Whether this mission has a wrong-face closure does not matter; the frames are the data.

## Analysis (fixed now)

`python3 eval/image_depth.py BAG`. Frames within 10 degrees of a face centre, in 2 m depth bins from 1 to 22 m. Per bin: median brightness and contrast of the rendered image, median ORB keypoints in the enhanced image, and the median RANSAC inliers between frames of different faces (cross) and between frames of the same face at least 60 s apart (same).

## Predictions

- **A holds** if brightness and ORB keypoints in the 20 to 22 m bin are both below their median over the 5 to 15 m bins by at least 20 %.
- **B holds** if the cross/same inlier ratio in the 20 to 22 m bin is at least 1.5 times its median over the 5 to 15 m bins.
- Both, either or neither can hold. Neither holding means the bottom is not explained by these two and the question stays open.
- One mission; whatever it shows is a description of this run, to be repeated before it is claimed.

## Outcome

Run 2026-10-25, bag `a50_imgL0_seed1_20261025_132041` (1,263 enhanced frames, 1,316 s, mission complete). Square-on frames per 2 m bin, medians:

| depth (m) | frames | brightness | ORB | cross inliers | same inliers | ratio |
|---|---|---|---|---|---|---|
| 4-6 | 36 | 54.4 | 52 | 0 | 0 | - |
| 6-8 | 34 | 53.6 | 132 | 3.5 | 11.0 | 0.32 |
| 8-10 | 64 | 54.1 | 175 | 7.0 | 16.0 | 0.44 |
| 10-12 | 68 | 53.6 | 190 | 10.0 | 17.5 | 0.57 |
| 12-14 | 61 | 53.3 | 202 | 8.5 | 14.5 | 0.59 |
| 14-16 | 56 | 53.6 | 174 | 7.5 | 8.0 | 0.94 |
| 16-18 | 47 | 53.6 | 230 | 11.5 | 15.5 | 0.74 |
| 18-20 | 52 | 54.0 | 231 | 12.0 | 20.0 | 0.60 |
| 20-22 | 77 | 53.9 | 246 | 11.0 | 19.0 | 0.58 |

- **A does not hold.** Brightness at 20-22 m is the same as over 5-15 m (53.9 against 53.6, +0 %); ORB keypoints are 41 % higher, not lower (246 against 174). The rendered image does not darken with depth over this range, and the bottom has the most features, not the fewest.
- **B does not hold.** The cross/same ratio at 20-22 m is 0.58 against a 5-15 m median of 0.57 (x1.01; the 4-6 m bin had no matches and no ratio, so the median is over 6-16 m). The most aliased bin in this run is 14-16 m (0.94), not the bottom.
- Neither holds, so the seabed-end concentration is not explained by these two and the question stays open. One mission, descriptive only. The inlier test is a proxy (OpenCV ORB plus fundamental-matrix RANSAC, not ORB-SLAM3's bag-of-words place recognition).

The mission itself had one loop closure, at 798 s, during the descent to face 3 at about 20 m: face-aliased, bearing turn -90.6 deg, radius kept (-4.6 %), so a 13th wrong-face closure at the seabed end (`closure_events.py`). The 5 m gate refused the jump (fused error 0.43 m at 805 s). But the bridge did not stay silent afterwards: its windows straddling the jump gave fits with scale 0.29, 0.51 ... 0.72, all rejected, until one reached 0.751 at 931 s and was accepted (theta +57.6 deg, residual 0.36 m) and kept being refitted. Fused error then rose from 0.44 m (1000 s) to 1.5 m (1100 s) and ended at 1.52 m; the SLAM-free reference ended at 0.05 m. True final position 3.4 m from home. The scale bound [0.75, 1.25] is not enough to reject a cross-fragment window whose scale drifts slowly into range. The Atlas was not saved (the known shutdown segfault).
