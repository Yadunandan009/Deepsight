# [9]. Conclusion

*Draft 2026-10-19. Section number is a placeholder. Every claim is one the
results sections support; the hedges are the ones in `section_limitations.md`.*

We asked what camera SLAM adds to the navigation of an inspection vehicle that
already has a velocity log, an inertial unit and a depth sensor, around a
four-fold symmetric offshore structure, and we scored the answer on the structure
the vehicle delivers as well as on its position.

In simulation, on a single lap, it added nothing we could measure and one hazard.
Dead reckoning alone held position to 0.1 to 0.5 m and reconstructed the
structure from the vehicle's own sonar to a median of 11 cm RMS against a sensor
floor of 7 cm. Fusing SLAM raised the position error in every valid recording
and, in the two recordings made on the corrected pipeline, the reconstruction
error to 0.38 and 1.8 m.

The hazard is a loop closure to the wrong face of the structure. Five of twelve
classified closures re-posed the estimate by 90 degrees (once 180) about the
structure's axis, keeping its radius, to a place the vehicle had never been; at
least 5 of 28 missions were affected. A vehicle that trusted SLAM through such a
closure would have rebuilt the structure to 3.8 to 7.5 m, and ORB-SLAM3's own map
is damaged in the same way in three of five missions measured. On a lap, a plain
check against dead reckoning catches every one. In an offline test with injected
drift, a check on the closure's length alone, which needs no dead reckoning,
keeps catching them after the plain checks have failed.

What this does not show is as important. It is one simulated scene with one SLAM
system, the symmetry is shown to match the closures' geometry but not to cause
them, the fused-filter result after the fix rests on two recordings, and the
checks have not been run in the loop. The next steps are to replicate the
corrected-pipeline result, to find the drift at which SLAM starts to help, and
to fly the same missions around a structure without the symmetry.
