# The path-length inflation is sampling noise (2026-10-16)

The last surviving item on the defect list dissolves under the same
treatment as the others: it was a property of how the quantity was
computed, not of the SLAM.

## The test

Summing `|dp|` over consecutive poses is biased upward by per-sample
noise. With independent error `sigma` and true motion `d` per step,

    E|dp| ~ sqrt(d^2 + 2 sigma^2)

The bias is fixed while `d` grows with the sampling interval, so the two
explanations separate cleanly under decimation:

- **noise** -> the ratio falls toward 1.0 as the interval grows
- **real excess motion** -> the ratio holds at every scale

`eval/path_scale.py` decimates the SLAM track and ground truth
identically and reports the ratio at each interval.

## The result: it falls, every time

| run | 0.07 s | 0.27 s | 1.1 s | ~9 s | implied noise |
|---|---|---|---|---|---|
| slamref seed 0 | 2.37 | - | - | **1.01** | 1.8 cm |
| slamref seed 1 | 1.83 | - | - | **1.03** | 1.4 cm |
| slamref seed 2 | 2.77 | 1.86 | 1.35 | **1.06** | 1.8 cm |
| slamref12 seed 0 | 1.90 | 1.27 | 1.13 | **1.09** | 1.3 cm |
| slamb30 seed 0 | 1.82 | - | - | **1.04** | 1.2 cm |

Monotone decay in every run, converging to **1.01-1.09**.

The implied per-sample pose noise is **1.2-1.8 cm**, against a true
motion of **1.0-1.3 cm per sample**. The vehicle moves about one
centimetre between frames at 14 Hz and the pose is uncertain by rather
more than that, so the integrated arc length roughly doubles. That is
arithmetic, not a defect.

## What is actually left

A genuine **1-9 % excess at long timescales**, which is small, varies by
run, and is unremarkable for a visual system. Nothing here needs
explaining.

Note this coexists with a global scale of 0.95-0.97 without
contradiction: scale is fitted to net displacement, path length is arc
length. SLAM slightly under-shoots where it ends up while slightly
over-shooting how far it travelled.

## Consequence for the earlier argument

`docs/MECHANISM_RESTATEMENT.md` section 3 argued against perceptual
aliasing on the grounds that "a rotation is an isometry, so it cannot
change path length" -- citing the 1.8-2.8x inflation as something a
rotation could not produce. **The logic is sound and the conclusion
stands, but the premise should not be used:** the inflation was not a
property of the error at all. Section 3 should rest on the scale
evidence (0.95-0.97, which a rotation also cannot produce) rather than
on path length.

## Status of the defect list

Everything that opened this investigation is now accounted for:

| symptom | verdict |
|---|---|
| no fixed map-to-world transform | artifact of the ENU/NED frame bug |
| theta wandering, dispersion 0.88-0.96 | same artifact; corrected 0.00-0.07 |
| degenerate global scale 0.06-0.44 | same artifact; corrected 0.95-0.97 |
| local scale 0.91 | same artifact; corrected 0.95-0.97 |
| path length 1.8-2.8x | sampling noise, this document |
| 90-degree re-lock signature (R3) | sample void; no re-lock events occurred |
| stereo scale starvation | refuted by the wide-baseline run |
| perceptual aliasing as drift cause | refuted; survives as a rare event |
| one loop closure at 89.0 deg | **real**, see LOOP_CLOSURE_FACE_CONFUSION.md |
| Atlas collapse during RISE | **real**, open; -69 deg gap also died |

Two real defects remain, both episodic rather than continuous: the loop
closure that matched the wrong face, and the Atlas fragmentation at the
RISE handoff.
