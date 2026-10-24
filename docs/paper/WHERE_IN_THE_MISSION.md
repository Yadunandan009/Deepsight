# Where in the mission the wrong-face closures fall (2026-10-22)

Descriptive, post hoc. Script: `eval/closure_where.py`; figure: `eval/figures/fig_where.pdf`; data: `eval/figures/closure_where.json`. 23 flights, 23 scored closures (12 wrong-face, 11 benign; 2 other-large and 2 unscorable not used).

## What the data say

- **Depth.** 9 of 12 wrong-face closures happened at 20 m depth or more (the scan runs 1 to 22 m, so this is the seabed end of the structure). Median depth 20.8 m for wrong-face against 18.0 m for benign (Mann-Whitney one-sided p = 0.005, no threshold needed). With the 20 m threshold: 9 of 12 against 1 of 11 (Fisher p = 0.002). The threshold was chosen after looking; at 19 m it is 10 of 12 against 1 of 11 (p = 0.0005) and at 18 m 11 of 12 against 5 of 11 (p = 0.024). Treat these as a description, not as tests.
- **Motion.** 10 of 12 wrong-face closures happened while the vehicle's vertical speed was under 0.25 m/s (9 of them with a bearing rate above 0.3 deg/s, 1 nearly still), 2 while descending. All 11 benign ones: 10 while descending at 0.26 to 0.44 m/s on face 0 at mid depth, 1 nearly still. The label "ring" in the data means only low vertical speed and a bearing rate above 0.3 deg/s; it is not the 9.5 m transit orbit (the wrong-face closures are at 20 m or more), so read it as the bottom turnaround or the start or end of a transit.
- **Face position.** Both classes sit within 4 degrees of a face centre, so this does not separate them.
- **What they point at.** For the 90-degree closures the place matched had been visited 145 to 222 s earlier at the same depth (the bottom of the previous face), and for the 180-degree ones 436 to 453 s earlier. In all 12 the vehicle had been at the matched bearing and depth earlier, so the closure points at a place the vehicle really visited, just not the one it was at.
- **Why the period looked different.** Early on, wrong-face closures fell mid-mission at the bottom (525 to 776 s, and 331 to 336 s in L4). Since 2026-10-20 they mostly fall at the return to face 0 (1018 to 1034 s), again at the bottom (20.6 to 21.1 m). The benign closures at the return are the ones that happen higher up (14 to 19 m). So the same event, the vehicle arriving at face 0, gives a benign closure at mid depth and a wrong-face one at the bottom.
- **L5.** No L5 flight had any closure at the bottom. Each had one closure, benign, at 18 m on the return.

## What this does and does not show

It shows when and where the failure happens. It does not show why. The stock structure is symmetric at every depth, so the CAD pair score cannot explain a depth effect. Two untested candidates:
1. light falls with depth in the simulator's water model (Jerlov 0.22), so the images are darker at the bottom;
2. at the bottom the view is dominated by the base and the seabed, which look alike on every face.

Neither can be tested on the recorded bags: no camera images were recorded. A test needs one run with images recorded (mean brightness and feature counts against depth), or a scenario with brighter water.

The result is also consistent with the symmetry reading: the L5 flights never produced a closure at the bottom at all, which is where the stock flights produced theirs.

## Caveats

- The classifier decides what counts as an event, and the benign class is defined by a short correction, so benign closures are by construction at places the vehicle is already at.
- 12 wrong-face events come from 9 flights, and in several flights (the L4 pair, the three slamref re-measures) the same controller and structure repeat, so the events are not independent.
- The depth threshold was set after seeing the data.
