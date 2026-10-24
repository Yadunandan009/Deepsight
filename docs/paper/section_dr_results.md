# [N]. Dead reckoning, and what fusing SLAM does to it

*Draft 2026-10-17. Results section. Evidence: `eval/analyze_anchorfree.py`,
`eval/filter_ab.py` for [N].1 and [N].2; `eval/sonar_recon.py`,
`eval/recon_batch.py`, `eval/fig_recon.py` for [N].3. Section, table and
figure numbers are placeholders.*

## [N].1 Dead reckoning on one lap

A single lap of the structure is 130 to 133 m of path in about 1150 to 1180 s.
Over that lap, dead reckoning from the DVL, inertial unit and pressure sensor
alone holds the horizontal position to a fraction of a metre (Table [D]). The
three DVL units differ because each carries its own fixed scale and
misalignment error, which is the point of drawing them from a seed: a single
unit is one sample from a wide spread, 0.10 to 0.54 m across eight seeds on one
fixed trajectory.

**Table [D].** Dead reckoning over one lap, with no SLAM, by DVL unit.

| DVL unit | final error (m) | per cent of path | RMS (m) |
|---|---|---|---|
| white noise only (no systematic error) | 0.04 | 0.01 | 0.03 |
| seed 0 | 0.11 | 0.08 | 0.10 |
| seed 1 | 0.34 | 0.26 | 0.20 |
| seed 2 | 0.51 | 0.37 | 0.36 |

The inertial unit's heading error is 0.064 degree on average with no growth in
all of these. With a yaw drift of 3 degrees per hour added, confirmed in the
filter at 3.00 degrees per hour, dead reckoning is still 0.20 m RMS over the lap.
That is because of the lap itself. A heading error displaces an estimate in
proportion to its distance from where heading was last known, and an orbit never
takes the vehicle far from where it began (in the 3 degrees per hour mission, at
most 37.9 m from the start and 3.8 m from it at the end): 0.96 degree over 131 m of straight line would be
2.2 m, against about 0.6 m at most here (0.96 degree at the 37.9 m
farthest point). **This is a result about single-lap
inspection missions, and it is the reason a map-based correction has little to
correct.**

## [N].2 Fusing SLAM

Seven recordings carry both the SLAM-free reference filter and the filter that
fuses SLAM, on the same flight with the same sensors, so they differ only in
whether SLAM is used. Table [F] compares them from the first SLAM pose onward.

**Table [F].** Horizontal position RMS against ground truth, from the first
SLAM pose, in the same flight.

| recording | axis conversion | dead reckoning (m) | fused (m) | ratio |
|---|---|---|---|---|
| 2026-10-14 14:08 | before | 0.34 | 3.20 | 9 |
| 2026-10-14 15:36 | before | 0.10 | 3.76 | 37 |
| 2026-10-14 20:03 | before | 0.40 | 4.43 | 11 |
| 2026-10-15 18:05 | before | 0.11 | 3.66 | 33 |
| 2026-10-15 20:41 (wide baseline) | before | 0.11 | 3.88 | 37 |
| 2026-10-16 16:00 (wrong-face closure) | **after** | 0.11 | 3.15 | 28 |
| 2026-10-16 19:07 (3 deg/hr gyro) | **after** | 0.22 | 1.07 | 5 |

In every one the fused filter is worse than the same filter without SLAM. **Six
of the seven predate the axis-conversion fix**, so their fused track carried a
mirrored alignment and their ratios cannot be taken as a measurement of SLAM.
The two that follow it give 28, in a mission with a wrong-face closure, and 5,
in a mission whose only closure was benign. In that second mission the ratio is
2.6 over the inspection proper (0.17 m against 0.45 m); the rest comes from the
final rise, where the map collapsed from 403 918 points to 236. A recording
excluded from the table, 2026-10-14 16:25, was flown with no systematic DVL
error (its measured gains are 1.0000), which makes its dead reckoning
near-perfect (0.02 m) and its ratio meaningless.

**Why a rougher measurement worsens a better estimate.** After alignment SLAM's
position is good to a few tenths of a metre on a clean stretch, against dead
reckoning's 0.1 to 0.4 m, and a measurement no better than the estimate it is
fused into cannot improve it. The bridge compounded this by declaring every pose
good to 0.22 m. Replaying its alignment offline against ground truth gave a true
error of 0.28 m on a clean mission and 1.64 m on the mission with the wrong-face
closure, which is 2 and 54 times the declared variance. The filter believed SLAM
more than it should.

**What this does and does not show.** It shows that, on a single lap, fusing SLAM
through this bridge did not help and usually hurt. It does not show that SLAM
cannot help: the experiment built to find the point where it starts to help
(3 degrees per hour of gyro drift) was void because dead reckoning stayed below
the 0.5 m registered as the criterion. Only two recordings postdate the fix, and
both are single seeds. A bridge that fits SLAM's scale and publishes an honest
covariance has been written and has never flown.

## [N].3 What the vehicle delivers: the reconstructed structure

Pose error is a means. An inspection vehicle delivers a model of the structure, so
we score that. The vehicle's own multibeam, a planar fan that becomes a
three-dimensional picture only as the vehicle sweeps up and down, is projected
into the world under each navigation source, and every point is measured against
the structure's CAD mesh. The set of returns is fixed: a return counts if it lies
within 0.5 m of the mesh under the true pose, and the same returns are then placed
by each source, so a difference between sources is pose error and nothing else.

**The floor.** With the true poses the reconstruction is 7.3 cm RMS in every one
of the 17 recordings scored (7.3 to 7.5 cm), consistent with the sonar's 10 cm
range noise. That is what no navigation source can beat.

**Dead reckoning builds the structure almost at the floor.** Across ten
recordings the reconstruction is 9.0 to 58.5 cm RMS, median 11 cm (Figure [R]).
The spread follows the DVL unit: the three dedicated dead-reckoning runs give 9,
16 and 20 cm, and the 58 cm recording is the one whose dead-reckoned position was 0.40 m
RMS. In the mission shown in Figure [P], after the wrong-face closure, 98 % of the
points lie within 25 cm of the CAD surface (99 % over the whole run).

**Fusing SLAM raises the error by one to two orders of magnitude.** In the five
fused-filter recordings made before the axis fix the reconstruction is 2.2 to
2.7 m (median 2.4 m); the run in which SLAM was in the loop against its own filter
gives 4.0 m. Those six fused tracks carry a mirrored alignment, so they show what
the defect cost, not what SLAM does. The two recordings after the fix give 0.38 m
in a mission with only a benign closure and 1.8 m in the mission with a wrong-face
closure, against 0.12 m and 0.10 m for dead reckoning in the same flights. In the
second, after the closure, only 6 % of the points lie within 25 cm of the CAD
surface (44 % over the whole run; Figure [P]c).

**A wrong-face closure ruins the model.** We built the structure as a vehicle would
have if it had trusted SLAM through the closure: the raw SLAM pose, an alignment
fitted only on the poses before the closure and held fixed, and a constant
rotation offset calibrated before the closure. Before the closure this gives
0.26 to 0.73 m. After it the reconstruction is **3.8 to 7.5 m RMS, in all five
wrong-face missions** (median 5.8 m), and in the mission of Figure [P] 1 % of the
points after the closure lie within 25 cm of the CAD surface (34 % over the whole
run). The picture is a second, rotated copy
of the structure laid across the first (Figure [P]d). That is the closure's 88
degrees about the structure's axis, seen in the product the inspection exists to
deliver.

**SLAM's own map is not a substitute.** ORB-SLAM3 also builds a map, and it is
published as a point cloud. We placed it in the world with a similarity fitted
between the SLAM trajectory and the true trajectory before the closure, held
it fixed, and measured every map point against the CAD in the same way as the
sonar (Figure [S], Table [M]; the true trajectory is used for the placement only,
so this scores the map, not what a vehicle could know). Before the closure the
map follows the structure but is noisy: 17 to 21 % of its points lie within 25 cm
of the CAD (median 0.7 to 0.9 m), against 99 % for the sonar reconstructed by dead
reckoning. Part of that is placement: if the map is instead placed as well as it
can be against the CAD (a best-case placement that uses the CAD), 46 to 66 % lie
within 25 cm. After the closure the map is damaged in three of the five missions:
the median distance is 2.5 to 4.1 m, and in the best case it stays at 2.1 to 4.2 m,
so this is not a placement effect. Duplicate copies of the structure appear where
there is no structure (Figure [S]c). In the other two missions the map is only
modestly worse (14 to 15 % within 25 cm, from 17 %), and in one of them (2026-09-22
23:09) the last map before ORB-SLAM3 reset it was taken 4 s after the closure, so
that row says little.

The same map in ten flights with no wrong-face closure is a control. Split at
the flight's benign closure (or at 1000 s where it had none), the map is
essentially unchanged: 16 to 24 % of its points within 25 cm before the split and
16 to 24 % after, the largest change being 1.3 percentage points, with a median
distance of 0.6 to 0.8 m throughout (Table [M], second part). The damage after a
wrong-face closure is therefore not a property of any later part of a mission.
In six of those flights the "after" snapshot is within 30 s of the split, so
the comparison is weak there; in four (2026-10-14 14:08, 15:36 and 20:03, and
2026-10-15 18:05) it is 45 to 170 s later and shows no change.

**What this does and does not show.**
- It shows that the pose errors above are not an abstract cost: they decide
  whether the model of the structure can be used.
- It does not show that a vehicle trusting SLAM would behave as the counterfactual
  does. That condition was constructed from the recordings, not flown. Its
  pre-closure rotation residual is 1.7 to 4.0 degrees in four missions and 11.7 in
  the fifth (23:23 on 2026-09-21), whose pre-closure error of 0.73 m comes from that
  calibration, not from SLAM.
- The absolute numbers are conditional on the CAD being the truth, on the
  sonar's Gaussian noise, and on the return set being defined under the true pose.
  The comparison between sources is not affected by the last.
- Only two fused recordings postdate the fix.

**Table [R].** Reconstruction RMS error against the CAD mesh, every recording.
With true poses it is 7.3 to 7.5 cm in all of them.

*(a) Dead reckoning alone, three DVL units, one lap each* (the unit with no
systematic DVL error gives 8 cm).

| DVL unit | reconstruction RMS |
|---|---|
| seed 0 | 9 cm |
| seed 1 | 16 cm |
| seed 2 | 20 cm |

*(b) Same flight, dead reckoning and the SLAM-fused filter.* The recording of
2026-10-14 16:25 is omitted: it was flown with no systematic DVL error, giving
8 cm for dead reckoning and 2.27 m fused.

| recording | axis fix | dead reckoning | SLAM-fused |
|---|---|---|---|
| 2026-10-14 14:08 | before | 29 cm | 2.52 m |
| 2026-10-14 15:36 | before | 10 cm | 2.17 m |
| 2026-10-14 20:03 | before | 0.59 m | 2.73 m |
| 2026-10-15 18:05 | before | 10 cm | 2.32 m |
| 2026-10-15 20:41 | before | 10 cm | 2.43 m |
| 2026-10-16 19:07 | after | 12 cm | 38 cm |
| 2026-10-16 16:00 (wrong-face closure) | after | 10 cm | 1.80 m |

*(c) Missions with a wrong-face closure, a vehicle that trusted SLAM through it
(counterfactual).* The last column is the residual of the rotation calibration
fitted before the closure; the 2026-09-21 row's pre-closure error comes from it.

| mission | closure at | before closure | after closure | rotation calibration residual |
|---|---|---|---|---|
| 2026-09-05 20:11 | 776 s | 28 cm | 3.82 m | 1.7 deg |
| 2026-09-21 23:23 | 642 s | 0.73 m | 5.17 m | 11.7 deg |
| 2026-09-22 00:01 | 561 s | 26 cm | 7.51 m | 1.7 deg |
| 2026-09-22 23:09 | 525 s | 0.54 m | 5.75 m | 4.0 deg |
| 2026-10-16 16:00 | 573 s | 35 cm | 6.60 m | 2.6 deg |

**Figure [P].** The structure reconstructed from the vehicle's own sonar in the
mission of 2026-10-16 16:00, as a three-dimensional point cloud, under four
navigation sources. The same sonar returns appear in every panel; only the pose
that places them changes. Colour is each point's distance to the CAD mesh,
clipped at 1 m; grey is the mesh. **(a)** True poses, 7 cm RMS over the whole
run. **(b)** Dead reckoning, 10 cm. **(c)** The SLAM-fused filter, 1.8 m: the
lattice is smeared. **(d)** A vehicle that trusted SLAM through the wrong-face
closure, 4.5 m: a second, rotated copy of the lattice lies across the first.

**Figure [E].** What Figure [P] means, for the same mission. **(a)** For each
source, the share of the reconstruction that lies within a given distance of the
CAD. With dead reckoning 98 % of the points are within 25 cm after the closure,
against 6 % for the SLAM-fused filter and 1 % for the vehicle that trusted SLAM.
**(b)** The same error through the mission, as RMS in 10 s windows. True poses
and dead reckoning stay between 7 and 12 cm throughout. The SLAM-fused filter
leaves them when the first SLAM pose reaches it (220 s) and runs at 0.2 to 0.5 m
before the closure; the vehicle that trusts SLAM runs at 0.1 to 0.6 m, its own
alignment and rotation being calibrated and not exact. At the wrong-face closure
(573 s) both SLAM-based curves jump by more than an order of magnitude, to several
metres, while dead reckoning does not move. Line styles are the same in both
panels.

**Figure [R].** The same measure across recordings. Each dot is one recording;
the black line is the median of its group. Groups are named on the axis, and
every dot has the same shape because only the group matters. The two lightest
groups are the fused filter before the axis conversion was fixed (their fused
tracks carry a mirrored alignment) and the run in which SLAM was in the loop
against its own filter. The last group is the whole-run RMS of the counterfactual
vehicle that trusted SLAM through a wrong-face closure. The numbers behind every
dot are in Table [R].

**Figure [S].** The structure as the vehicle's sonar and as ORB-SLAM3's own map
show it, in the mission of 2026-10-16 16:00. Colour is each point's distance to
the CAD mesh, clipped at 1 m; grey is the mesh; each panel shows 22 000 points
drawn at random. **(a)** The sonar placed by dead reckoning: 99 % of its points
are within 25 cm. **(b)** ORB-SLAM3's map, the last snapshot before the closure,
placed with a similarity fitted to the true trajectory before the closure: 21 %
within 25 cm. **(c)** The same map, the largest snapshot after the wrong-face
closure, with the placement held: 7 % within 25 cm, and copies of the structure
where none exists. Only 60 % of that map's points fall inside the plotted volume.

**Table [M].** ORB-SLAM3's map against the CAD, the five wrong-face missions.
"Before" is the last snapshot before the closure, "after" the largest snapshot
after it. The first pair of columns uses the trajectory-based placement; the second
pair places the map as well as possible against the CAD.

| mission | within 25 cm, before | within 25 cm, after | median distance, after | best placement: within 25 cm, after | best placement: median, after |
|---|---|---|---|---|---|
| 2026-09-05 20:11 | 18.5 % | 5.2 % | 2.50 m | 8.2 % | 2.13 m |
| 2026-09-21 23:23 | 20.2 % | 8.2 % | 3.04 m | 12.9 % | 2.84 m |
| 2026-09-22 00:01 | 16.6 % | 14.5 % | 1.04 m | 36.3 % | 0.47 m |
| 2026-09-22 23:09 (snapshot 4 s after closure) | 17.5 % | 13.7 % | 1.17 m | 34.4 % | 0.51 m |
| 2026-10-16 16:00 | 20.9 % | 7.3 % | 4.13 m | 12.5 % | 4.18 m |

*Second part: the control, ten flights with no wrong-face closure.* Snapshots
before and after the split time; the "after" row repeats the "before" row to
within 1.3 points.

| flight | split at | within 25 cm, before | within 25 cm, after | median, before | median, after | best placement: within 25 cm, before |
|---|---|---|---|---|---|---|
| 2026-10-14 14:08 | 1000 s | 22.1 % | 21.9 % | 0.70 m | 0.70 m | 56.7 % |
| 2026-10-14 15:36 | 1056 s | 23.1 % | 22.2 % | 0.65 m | 0.73 m | 62.4 % |
| 2026-10-14 16:25 (no DVL error) | 1034 s | 23.4 % | 23.4 % | 0.66 m | 0.67 m | 55.8 % |
| 2026-10-14 20:03 | 1000 s | 20.4 % | 19.8 % | 0.75 m | 0.77 m | 33.3 % |
| 2026-10-15 18:05 | 1000 s | 22.6 % | 22.9 % | 0.69 m | 0.68 m | 55.1 % |
| 2026-10-15 20:41 | 1000 s | 24.2 % | 23.8 % | 0.61 m | 0.62 m | 63.0 % |
| 2026-10-16 19:07 | 1015 s | 22.9 % | 22.2 % | 0.62 m | 0.64 m | 66.0 % |
| 2026-09-05 14:16 | 1028 s | 17.0 % | 17.5 % | 0.80 m | 0.79 m | 57.9 % |
| 2026-09-05 17:38 | 1014 s | 18.5 % | 18.5 % | 0.71 m | 0.70 m | 68.1 % |
| 2026-09-12 13:27 | 1027 s | 17.7 % | 16.4 % | 0.78 m | 0.83 m | 51.8 % |

The trajectory-based placement finds a scale of 0.89 to 0.96 in every mission,
against 0.97 to 1.01 for the best-case placement (checked for the five
wrong-face missions; the ten control flights show the same pre-closure gap: scale 0.89 to 0.98, best-case within 25 cm 33 to 68 %); that gap is why the
trajectory-based placement scores worse than the best-case one, and its cause is
not established (it may be the SLAM trajectory's own drift before the closure).

---

### Status note (not for the manuscript)

**Settled:** Table [D] (`analyze_anchorfree.py` on `a50_dr_seed0_20261009_221008`,
`a50_dr_seed1_20261012_173121`, `a50_dr_seed2_20261012_191733`, and the
white-noise baseline `a50_dr_seed0_20261009_210804`); Table [F] and the 2.6
(`filter_ab.py`, post-first-SLAM-pose window). **The ratios in Table [F] for
the six earlier rows are not a SLAM result;** they must not be quoted in the
abstract or the introduction. The post-fix evidence is two recordings.
**Needs the planned flights:** a clean post-fix replication with three seeds, for
both [N].2 and [N].3.
**Excluded, and why:** `a50_dr_seed1_20261012_200040` (SLAM was in the loop,
this is the pre-fix circular-alignment run behind the 41 m figure);
`a50_slamref_seed2_20261014_162539` (no systematic DVL error).
**Unchecked:** the 0.10 to 0.54 m spread across eight seeds is from a comment in
`dvl_a50.yaml`; I did not reproduce it. The 0.28 m / 1.64 m offline replay is in
`docs/ALIGNMENT_WANDER.md` and used the bridge's alignment, not a live bridge.

**[N].3 settled:** every number is from `eval/figures/recon_results.json`
(17 recordings, none failed); ranges and medians recomputed from it, not read
off the plot. **[N].3 unchecked:** the plan view uses 30 000 random points per
panel, so panel RMS values are the full-set values from the cache, not
recomputed from the plotted subset. The reconstruction metric has not been
compared with an independent implementation; its sanity check is that true poses
give 7.3 cm against a stated 10 cm sensor noise.
