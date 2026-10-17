# 3. Testbed, navigation architecture and protocol

*Draft 2026-10-17. Every parameter below was read from the code or
configuration in this repository on that date; file names are given in the
status note so each can be re-checked. Section numbers are placeholders.*

## 3.1 Simulator and scene

The testbed is the Stonefish marine robotics simulator (v1.3) with its ROS 2
interface, running on ROS 2 Jazzy [32]. The vehicle is a BlueROV2-class
vectored-thruster vehicle. The scene is 25 m of water (Jerlov type 0.22,
density 1031 kg m^-3, overhead sun) over a flat seabed. It contains two
jacket-type structures built from CAD meshes: the structure under inspection
at (20, 0) m, which rises from the seabed through the surface, and a second
structure at (-20, 0) m. The vehicle starts at the origin, between them and
facing the first. Ground truth is the simulator's own pose of the vehicle.

## 3.2 Sensors

- **Stereo cameras.** Two cameras, 640 x 480, 75 degree horizontal field of
  view, 0.145 m baseline, mounted forward. The simulator renders them at
  30 Hz; the SLAM system processes them at 12 Hz.
- **Doppler velocity log.** Stonefish's own model is zero-mean white noise
  (1.5 mm s^-1), which integrates as a random walk and is not what limits a
  real unit. We add the two terms that do: a per-axis scale-factor error and a
  small rotation between the true beam frame and the assumed mount frame, so
  that the measured velocity is **v_meas = S dR v**, with S = diag(1 + e_x, 1 +
  e_y, 1 + e_z). Both are fixed properties of a unit and therefore integrate
  linearly with distance and never average out. The scale error is drawn with
  a standard deviation of 1.01 % per axis, the long-term velocity accuracy
  quoted for the Nortek DVL A50, and the misalignment with 0.5 degree per
  axis, a typical mounting and beam-alignment budget. Both are drawn once per
  run from an integer seed and held, so a seed is one physical unit; the
  published covariance is deliberately not inflated to cover them, since a real
  unit's covariance describes only its random error.
- **Inertial unit.** Attitude noise 0.1 degree and angular-rate noise
  1 degree s^-1, with an optional constant yaw-drift rate. The default
  configuration has no yaw drift; one experiment adds 3 degrees per hour.
- **Pressure sensor** for depth.
- **Multibeam sonar.** A forward-looking horizontal fan: 90 degrees, 128
  beams, 10 Hz, range 0.25 to 50 m, 0.1 m range noise. It returns a single
  planar slice at the vehicle's depth, so a three-dimensional picture of the
  structure exists only because the vehicle sweeps up and down it.

## 3.3 Navigation architecture

**Primary filter.** A 15-state extended Kalman filter (robot_localization
[24]) at 50 Hz fusing DVL velocity, IMU attitude and angular rate, and depth
from pressure. SLAM enters through a single input, its horizontal position;
its yaw is not used.

**SLAM.** Stock ORB-SLAM3 [7] in stereo mode, run in a container with a ROS 2
wrapper and unmodified in its place-recognition and map-merging behaviour. The
settings are 1500 features per frame, scale factor 1.2, 8 pyramid levels,
stereo baseline 0.145 m and a close-point threshold of 40 baselines, which puts
the boundary between triangulated and bearing-only points at 5.8 m. We keep the
default multi-map (Atlas) representation, so tracking loss starts a new map and
a recognised place can merge maps.

**SLAM-to-filter bridge.** The wrapper publishes in the east-north-up
convention and the stack is north-east-down; the bridge converts on ingest. The
conversion negates y and z, which is a rotation by 180 degrees about x in three
dimensions but flips the handedness of the horizontal plane, so no rotation-only
alignment can absorb it. The bridge then fits a horizontal rigid transform from
the SLAM frame to the world frame by closed-form least squares over a trailing
window of about 75 s, refitting every 5 s with exponential smoothing, and
rejects poses that disagree with the filter by more than 5 m or that follow a
map-size collapse. A SLAM jump of more than 5 m resets the alignment. The
bridge fixes the scale at one and publishes a constant position variance of
0.05 m^2. The axis conversion above was added on 2026-10-16. Recordings made
before then fed the filter an unconverted, mirrored SLAM pose. The closure
analysis reads the SLAM topic and converts it on read, so it is unaffected, but
fused-filter results from those runs are not comparable with later ones. The
two recordings used for fused-filter results here were both made on 2026-10-16,
with the conversion in place.

**SLAM-free reference filter.** A second instance of the same filter without
the SLAM input. The bridge fits its alignment against this filter, not against
the filter that consumes the result. Fitting an alignment against the output of
the filter that uses it is unfalsifiable: the alignment is fitted to a quantity
it determines, so it agrees with itself whether or not it is right.

## 3.4 Mission and controller

The controller steers on the primary filter. It holds a 17 m standoff using
only the minimum sonar return as a range signal, and computes the bearing to the
structure from its known position and the filter's, with a one-time yaw
calibration at the start that assumes the vehicle spawns facing it. The mission
descends to depth, closes to the standoff, then visits four faces in turn. At
each it sweeps vertically between 22 m and 1 m depth and back, and between
faces it orbits at about 9.5 m depth through an arc of 69 to 90 degrees. After
the fourth face it returns to the first face, rises, and returns home. A
mission is about 1150 to 1180 s and 131 to 133 m of path.

## 3.5 Ground truth, and what navigation can and cannot see

Ground truth is used only to score, and in two analysis steps described below:
fitting the alignment used to place a closure in the world, and defining the set
of sonar returns that hit the structure. No navigation source sees it. One
caveat on the older recordings: until 2026-10-08 the primary filter was
anchored to ground-truth position, so the seven recordings of September were
flown by a controller steering on a better estimate than a GPS-free vehicle
would have. SLAM itself uses only the cameras, so the closures it makes do not
depend on that, but the flown trajectories do.

## 3.6 Protocol

Experiments are pre-registered in the repository before the runs: the arms, the
seeds, the predictions and the criteria that would falsify them. The recording
script refuses to record a run whose arm name contradicts the live system (the
DVL seed, the stereo baseline, whether SLAM is publishing). Where a prediction
failed or a run was void, the registration records it as such.

---

### Status note (not for the manuscript)

**Sources for each fact:** `src/stonefish_bluerov2/scenarios/bluerov2_turbine.scn`
and `turbine1.scn`, `turbine2.scn` (scene, positions, seabed at z = 25,
water); `scenarios/bluerov2_imudrift.scn` (sensors, IMU noise, multibeam,
cameras); `scripts/dvl_bridge.py` and `dvl_a50.yaml` (DVL model and values);
`ekf.yaml` (filter inputs); `slam_params/bluerov2_stereo.yaml` (SLAM settings);
`scripts/slam_pose_bridge.py` (bridge; the version in the repository now
differs from the one the recordings used, see below);
`scripts/bluerov2_autonomous_controller.py` (mission); `record_run.sh`.

**Check before submission:**
1. **The bridge described is the version the recordings used.** The repository
   now holds a newer one that fits scale and publishes a covariance derived from
   the fit residual (2026-10-16). It has never flown. The ENU/NED conversion
   landed in commit 7d77b4c the same day; recordings before it have a mirrored
   fused-filter track. Of the nine recordings that have both filters, only two
   (2026-10-16 16:00 and 19:07) are post-conversion. Clean fused-filter results
   need the planned re-measurement flights. If new flights use the new bridge, this paragraph and every result
   from those flights must say so.
2. **"Stock" ORB-SLAM3** means the library's own place-recognition and merging
   logic; the container wrapper and parameter file are ours.
3. The DVL misalignment budget (0.5 degree) is described as typical from the
   repository's own comment; it has no citation.
4. The two-structure scene is a property of the scenario; whether the second
   structure ever contributes SLAM matches is not examined.
5. Cross-references to the closure-classification section and to the
   pre-registration documents are by name only.
6. **Bag names are not reliable identifiers of the DVL unit.** Checked
   2026-10-17 by the measured per-axis gain of each recording. `a50_slamref_seed2_20261014_162539`
   has gains of 1.0000 on all three axes, i.e. no systematic DVL error, though
   its name says seed 2; it is excluded from dead-reckoning statistics. Two
   recordings named seed 2 and two named seed 1 have visibly different gains
   from the dedicated dead-reckoning runs of the same name. Identify a unit by
   its fitted 3x3 operator, as the pre-registration does, not by its label.
