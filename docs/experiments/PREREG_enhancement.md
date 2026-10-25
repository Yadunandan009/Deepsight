# Pre-registration: does a better underwater enhancer give SLAM better features?

Written 2026-10-25, before any candidate was run on the frames. Offline only; no flight.

## Question

The current front end (`image_enhancement_node.py`: bilateral denoise, gamma 0.7, CLAHE on L with clip 4.5 and 6x6 tiles, two-scale unsharp mask) gives about 175 to 250 ORB keypoints per frame at 8 to 22 m in the jerlov 0.22 water of `bluerov2_turbine.scn`, against the 1500 ORB-SLAM3 asks for (image-depth run, 2026-10-25). Would an enhancer built for underwater images give more, and more repeatable, features? It is not expected to help wrong-face closures; the check below makes sure it does not make them worse.

## Data

The 1,263 rendered left-camera frames (`/bluerov2/left/image_color_rec`, 640x480, about 1 Hz) and the recorded enhanced frames of bag `a50_imgL0_seed1_20261025_132041`, with ground truth for depth and face.

## Methods compared

1. **raw**: the rendered frame, no enhancement.
2. **current**: the live chain, run through `ImageEnhancementNode.enhance` itself (checked against the recorded enhanced frames).
3. **fusion**: Ancuti et al. 2018, colour balance and fusion: red-channel compensation and grey-world white balance, then a gamma-corrected and a sharpened version, fused through a 5-level Laplacian pyramid with Laplacian-contrast, saliency and saturation weights.
4. **udcp**: underwater dark channel prior (Drews et al. 2013): dark channel over green and blue, 15 px patch, guided-filter transmission, floor 0.1, omega 0.95.

The candidates' parameters are those of the papers and are not tuned on these frames.

## Metrics (fixed now)

Frames square-on to a face (within 10 degrees of a face centre), 6 to 22 m. ORB with ORB-SLAM3's settings (1500 features, scale 1.2, 8 levels, FAST threshold 20). The same sampled pairs for every method:

- **ORB**: median keypoints per frame.
- **same**: median RANSAC inliers between frames of the same face at least 60 s apart (true revisits).
- **cross**: median RANSAC inliers between frames of different faces.
- **ratio**: cross / same.
- **time**: mean milliseconds per 640x480 frame on this laptop, single process.

## Decision rule

A candidate beats **current** only if all of these hold:

- **same** is at least 1.2 times current's;
- **ORB** is at least current's;
- **ratio** is no more than current's plus 0.05;
- **time** is at most 40 ms per frame (the cameras run at 12 Hz, one enhancer process per camera).

A winner does not go into the paper's pipeline as is. It becomes a new pipeline period with its own flights, never pooled with the flights already flown. One mission and one water setting; repeat before claiming anything general.

## Outcome

Run 2026-10-25 (`python3 eval/enhance_test.py BAG`), 459 square-on frames at 6 to 22 m, 300 sampled pairs of each kind, the same pairs for every method. The stand-in for the live node reproduces the recorded enhanced frames exactly (mean grey difference 0.00).

| method | ms/frame | ORB | same | cross | ratio |
|---|---|---|---|---|---|
| raw | 0 | 0 | 0 | 0 | - |
| current | 5.9 | 202 | 10.0 | 7.0 | 0.70 |
| fusion | 51.1 | 19 | 0 | 0 | - |
| udcp | 10.9 | 17 | 0 | 0 | - |

**Neither candidate beats current.** Both fail on same-face matches and keypoints, and fusion is also too slow (51 ms against the 40 ms limit). The reason is visible in the frames: the rendered image has no red at all (mean R = 0) and low local contrast (grey sd 17), so FAST at threshold 20 finds no corners in the raw frame. Fusion and UDCP are colour-restoration methods; they correct the cast but do not stretch local contrast, and that is what the corner detector needs. CLAHE does.

Sensitivity, not the registered metric: ORB-SLAM3 falls back to FAST 7 in grid cells where 20 finds nothing, which the proxy above does not. With FAST 7 throughout:

| method | ORB | same | cross | ratio |
|---|---|---|---|---|
| current | 1339 | 14.0 | 13.0 | 0.93 |
| fusion | 243 | 10.5 | 7.0 | 0.67 |
| udcp | 167 | 11.0 | 8.0 | 0.73 |

Current still has the most same-face matches, so the conclusion stands. But note what the low-threshold features are: current's extra 1,100 keypoints add almost as many cross-face matches as same-face ones (ratio 0.93), i.e. they are mostly features every face shares. Visible banding in the CLAHE output's open-water background (an 8-bit gradient stretched at clip 4.5) is one likely source, and is worth a look on its own. Untested.

One mission, one water setting (jerlov 0.22). Not repeated.
