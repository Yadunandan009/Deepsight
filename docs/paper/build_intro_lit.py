#!/usr/bin/env python3
"""Build the Introduction + Literature Survey as an editable .docx.

Written to the contribution fixed in docs/PAPER_EXECUTION_PLAN.md S1, not to
a new framing. Prose is drafting-quality and meant to be revised; the claims
it makes about our own results are restricted to what the bags support as of
2026-10-14, and anything still open is confined to the status note at the end
rather than asserted in the body.
"""
from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Pt, RGBColor

d = Document()
for style, size in (("Normal", 11),):
    d.styles[style].font.name = "Calibri"
    d.styles[style].font.size = Pt(size)


def h(text, level):
    d.add_heading(text, level=level)


def p(text, italic=False, align=None):
    par = d.add_paragraph()
    run = par.add_run(text)
    run.italic = italic
    if align:
        par.alignment = align
    par.paragraph_format.space_after = Pt(8)
    return par


def note(text):
    par = d.add_paragraph()
    run = par.add_run(text)
    run.italic = True
    run.font.color.rgb = RGBColor(0xA0, 0x30, 0x00)
    par.paragraph_format.space_after = Pt(8)
    return par


# ----------------------------------------------------------------- title
t = d.add_heading("Symmetry-Aware Visual Navigation for Autonomous Inspection "
                  "of Offshore Jacket Structures", level=0)
t.alignment = WD_ALIGN_PARAGRAPH.CENTER
p("Working title. Draft — Introduction and Literature Survey only.",
  italic=True, align=WD_ALIGN_PARAGRAPH.CENTER)
note("Drafted 2026-10-14. Every bracketed citation is from recall and MUST be "
     "checked against the actual paper before submission — authors, venue, "
     "year and, above all, that the paper says what it is cited for. The "
     "offshore-wind and industry-statistics citations are the least reliable "
     "and should be replaced with sources you have read. See the status note "
     "at the end for which of our own results are settled and which are not.")

# ------------------------------------------------------------ 1. intro
h("1. Introduction", 1)

p("Fixed-bottom offshore wind has moved from demonstrator scale to a core "
  "component of electricity supply in Europe and East Asia, and with it has "
  "come a maintenance obligation that grows with the installed base rather "
  "than with new construction. A jacket or monopile foundation must be "
  "surveyed on a recurring cycle for scour, marine growth, coating loss, "
  "weld condition and anode depletion. The work is done below the waterline, "
  "frequently in turbid coastal water, and it is today performed largely by "
  "work-class remotely operated vehicles flown by a pilot from a support "
  "vessel. The vessel, not the vehicle, dominates the cost, and the vessel is "
  "needed chiefly because a human is in the loop. Removing the pilot — "
  "replacing a teleoperated ROV with a resident or free-swimming autonomous "
  "underwater vehicle that executes the survey itself — is therefore the "
  "single change with the largest effect on inspection economics [1], [2].")

p("Autonomy of that kind is limited by navigation rather than by perception "
  "or control. An inspection trajectory is not a transit: the vehicle must "
  "hold a commanded standoff from a steel structure, typically of the order "
  "of ten to twenty metres for a sonar-and-camera survey and a few metres for "
  "close visual inspection, while traversing the asset face by face. A "
  "position error that would be irrelevant during transit becomes a collision "
  "with a structural member, and a heading error of the wrong magnitude puts "
  "the sensor payload on the wrong face entirely. The vehicle must therefore "
  "know where it is, continuously, to a tolerance set by the structure's own "
  "geometry.")

p("Underwater, it cannot be told. Radio-frequency positioning does not "
  "propagate: GNSS is unavailable below the surface, and the usual "
  "substitutes are acoustic. Long-baseline positioning requires a "
  "pre-deployed and surveyed array of seabed transponders; ultra-short-"
  "baseline positioning requires a surface vessel station-keeping above the "
  "vehicle with a hull-mounted transceiver [3], [4]. Both work, both are "
  "standard industrial practice, and both reintroduce precisely the surface "
  "asset whose elimination motivated the autonomy in the first place. A "
  "genuinely autonomous inspection vehicle must navigate from onboard sensing "
  "alone.")

p("Two onboard modalities are available and neither is sufficient by itself. "
  "A Doppler velocity log, integrated with attitude from an inertial "
  "measurement unit and depth from a pressure sensor, gives dead reckoning "
  "whose error is bounded in the vertical and unbounded in the horizontal. "
  "Crucially, the dominant horizontal terms are not white noise but fixed "
  "properties of the installed unit — a scale-factor error on each beam and a "
  "mechanical misalignment between the transducer frame and the vehicle frame "
  "— so the resulting position error grows linearly with distance travelled "
  "rather than with the square root of elapsed time [4], [5]. No amount of "
  "averaging removes it. Visual simultaneous localisation and mapping, by "
  "contrast, can in principle bound the error completely: a vehicle that "
  "recognises a place it has already mapped can close a loop and redistribute "
  "the accumulated drift across the trajectory [6], [7]. For an inspection "
  "mission this is an unusually good fit, because the mission is a loop by "
  "construction — the vehicle orbits a structure and returns to where it "
  "began.")

p("That argument is the standard one, and it contains a flaw specific to the "
  "asset class. Offshore jacket structures are built with four-fold symmetry "
  "about their vertical axis, not by coincidence but because a symmetric "
  "truss distributes environmental loading from any heading equally. The four "
  "faces are, to engineering tolerance, geometrically interchangeable. A "
  "place-recognition system — whether it matches appearance through a visual "
  "vocabulary or matches geometry through point-cloud registration — is "
  "therefore being asked to distinguish between views that the structure's "
  "designer deliberately made identical. The failure is perceptual aliasing, "
  "which is well known in general terms [8], [9]; what is specific here is "
  "that it is predictable in advance, from the engineering drawing, before "
  "the vehicle enters the water, and that its geometric signature is known: "
  "an alignment error that is a multiple of ninety degrees.")

p("This paper takes the position that a known symmetry should be exploited "
  "rather than suffered. An inspection vehicle, unlike a general-purpose SLAM "
  "platform, always knows what it is inspecting; a CAD model or survey "
  "drawing of the asset exists before the mission is planned, and generic "
  "SLAM discards that prior. We quantify the symmetry of the target as an "
  "a-priori index computed from the model, show that the interchangeability "
  "it predicts is measurable in the vehicle's own sensor data, and propose "
  "that corrections whose geometry corresponds to a symmetry operation of the "
  "structure be arbitrated by a modality that is blind to appearance — dead "
  "reckoning and depth, which cannot be fooled by a face that looks like "
  "another face.")

p("Establishing any of this requires a measurement apparatus that can be "
  "trusted, and a substantial part of the work reported here was concerned "
  "with making one. We find that the conventional way of fusing a visual SLAM "
  "pose into a navigation filter — estimating the rigid transform between the "
  "SLAM map frame and the world frame by correlating the SLAM trajectory "
  "against the filter's own output, while that filter consumes the result — "
  "is not merely inaccurate but unfalsifiable: the alignment is fitted "
  "against a quantity it determines, so it agrees with itself regardless of "
  "whether it is right. We show that opening this loop changes the reported "
  "error by more than an order of magnitude, and we adopt a SLAM-free "
  "reference filter as a standing instrument rather than a one-off diagnostic.")

p("The contributions of this paper are:")
for c in (
    "A simulation testbed for GNSS-denied inspection of a symmetric offshore "
    "structure, in which the Doppler velocity log carries a datasheet-grounded "
    "systematic error model — per-unit scale factor and beam misalignment drawn "
    "once from a seed — rather than the zero-mean white noise typically "
    "simulated, so that dead-reckoning error grows with distance as it does at sea.",
    "Identification and measurement of an observability defect in the common "
    "loosely coupled SLAM-to-EKF integration pattern, together with a "
    "SLAM-free reference filter that makes the alignment falsifiable, and "
    "evidence that the defect inflates reported navigation error by a factor "
    "of more than twenty.",
    "A model-derived symmetry index for the inspection target, and measurement "
    "of the geometric interchangeability of jacket faces independently from "
    "sonar and from CAD.",
    "Characterisation of the failure mode of an off-the-shelf stereo SLAM "
    "front end on a four-fold symmetric subsea structure, reported from the "
    "SLAM system's own diagnostics rather than inferred from its output.",
    "A symmetry-aware cross-modal gate that rejects corrections corresponding "
    "to a symmetry operation of the known structure, and an evaluation under a "
    "pre-registered multi-seed protocol with falsification criteria fixed "
    "before the runs were executed.",
):
    d.add_paragraph(c, style="List Bullet")

p("Section 2 reviews underwater navigation, visual SLAM and its underwater "
  "difficulties, place recognition, and current evaluation practice, and "
  "identifies the gap this work addresses. Section 3 describes the vehicle, "
  "the simulated environment and the sensor error models. Section 4 sets out "
  "the navigation architecture and the symmetry-aware gate. Section 5 gives "
  "the experimental protocol. Section 6 reports results and Section 7 "
  "discusses their limits.")

# -------------------------------------------------- 2. background
h("2. Background and Related Work", 1)

h("2.1 Navigation without a position fix", 2)
p("Underwater navigation is conventionally divided by whether an external "
  "position reference is available. Acoustic positioning supplies one. "
  "Long-baseline systems trilaterate against a surveyed array of seabed "
  "transponders and deliver metre-or-better accuracy that does not degrade "
  "with time, at the cost of deploying, calibrating and recovering the array. "
  "Ultra-short-baseline systems place a transceiver on a surface vessel and "
  "measure range and bearing to a vehicle-mounted beacon, trading array "
  "logistics for a persistent surface presence and an accuracy that degrades "
  "with slant range. Reviews by Kinsey et al. [3] and Paull et al. [4] remain "
  "the standard treatments and set out the trade space in detail. Both "
  "families are mature and neither is compatible with the economic argument "
  "for autonomy, since each restores an asset at the surface or on the seabed "
  "whose cost the autonomy was intended to avoid.")
p("The alternative is to navigate from proprioception and from what the "
  "vehicle can itself perceive. This is the regime addressed here, and it is "
  "where the accuracy requirement of close-standoff inspection collides with "
  "the error characteristics of the available sensors.")

h("2.2 Dead reckoning and the structure of its error", 2)
p("The Doppler velocity log is the workhorse of underwater dead reckoning. By "
  "measuring the Doppler shift of acoustic returns from the seabed along "
  "typically four slanted beams, it resolves vehicle velocity in the "
  "instrument frame; combined with attitude and heading from an inertial "
  "sensor and with depth from pressure, it yields a dead-reckoned position "
  "whose vertical component is effectively bounded and whose horizontal "
  "component is not [4], [5].")
p("The way that horizontal error grows matters for how it should be "
  "simulated, and it is frequently misrepresented. Treating the velocity "
  "measurement as corrupted by zero-mean white noise produces a random walk "
  "whose error grows as the square root of elapsed time and whose sign "
  "wanders. Real instruments are dominated instead by two terms that are "
  "fixed for a given unit and installation: a scale-factor error, quoted by "
  "manufacturers at the order of one per cent of measured velocity for "
  "commercial units, and a misalignment between the transducer and vehicle "
  "frames of the order of a degree. Both integrate linearly with distance "
  "travelled and neither averages out. A simulation that omits them will "
  "report dead-reckoning performance that cannot be reproduced at sea, and "
  "will make any SLAM aiding appear less necessary than it is. We adopt a "
  "model in which both terms are drawn once per simulated unit from a seed, "
  "so that a seed sweep samples the population of instruments rather than the "
  "noise on one.")

h("2.3 Visual SLAM and the ORB-SLAM family", 2)
p("Feature-based visual SLAM reached practical maturity with ORB-SLAM "
  "[6], which combined ORB features [10] with a covisibility graph, local "
  "bundle adjustment, and appearance-based loop closure over a bag-of-binary-"
  "words vocabulary [11]. ORB-SLAM2 [12] extended it to stereo and RGB-D, "
  "removing the scale ambiguity of the monocular case — a stereo rig with a "
  "calibrated baseline observes metric scale directly. ORB-SLAM3 [7] added "
  "visual-inertial operation and, most relevantly here, the Atlas multi-map "
  "representation: when tracking is lost the system begins a new map rather "
  "than attempting to recover within the old one, and merges maps later if a "
  "place is recognised across them. This is a considerable robustness gain in "
  "environments where tracking loss is routine, but it has a consequence that "
  "matters for anyone consuming the pose output, namely that the frame in "
  "which poses are expressed is not guaranteed to persist across a mission.")
p("Direct methods such as LSD-SLAM [13] and DSO [14] dispense with features "
  "and optimise photometric error, which can be advantageous in low-texture "
  "scenes but is sensitive to illumination change and to the non-Lambertian "
  "effects common underwater. Visual-inertial odometry systems such as "
  "VINS-Mono [15] tightly couple an IMU and achieve strong short-horizon "
  "accuracy, but without loop closure they remain odometry: the error is "
  "bounded in rate, not in magnitude. Graph back-ends such as iSAM2 [16] "
  "provide the incremental optimisation machinery these systems rest on.")

h("2.4 Visual SLAM underwater", 2)
p("Transplanting these systems below the waterline is not routine, and the "
  "reasons are physical rather than algorithmic. Wavelength-dependent "
  "attenuation removes the red channel within metres and compresses contrast; "
  "suspended particulate backscatters artificial illumination into a veiling "
  "glow; and the scene itself — a painted steel member, a sandy seabed, a "
  "uniform field of marine growth — frequently lacks the corner structure "
  "that ORB and similar detectors require. Köser and Frese [17] give a "
  "thorough account of these challenges, and Hidalgo and Bräunl [18] survey "
  "the underwater SLAM literature. Image enhancement is commonly applied as a "
  "pre-processing stage, with colour-balance-and-fusion methods [19] and "
  "learned enhancement trained on benchmark datasets [20] both in use; "
  "enhancement improves feature yield but does not create structure where the "
  "scene has none.")
p("Systems designed for the domain rather than adapted to it tend to add "
  "modalities. SVIn2 [21] fuses sonar, visual, inertial and depth measurements "
  "in a tightly coupled estimator and is among the strongest demonstrated "
  "underwater SLAM systems. Datasets such as AQUALOC [22] have made "
  "comparative evaluation on real subsea imagery possible. The present work "
  "deliberately uses an unmodified off-the-shelf stereo front end, because the "
  "claim concerns a property of the inspection target rather than of the SLAM "
  "implementation, and should transfer to any system with an appearance- or "
  "geometry-based place recogniser.")

h("2.5 Place recognition and perceptual aliasing", 2)
p("Loop closure rests on place recognition, and place recognition has a known "
  "failure mode. Appearance-based methods — FAB-MAP [23], the DBoW2 vocabulary "
  "used in the ORB-SLAM family [11] — represent an image by the visual words "
  "it contains and match it against a database. Two physically distinct places "
  "that contain the same words are, to such a system, the same place. Lowry et "
  "al. [8] survey the field and treat perceptual aliasing as one of its "
  "central problems. Geometric verification is the standard defence: a "
  "candidate match must admit a consistent rigid transform, tested by RANSAC "
  "or by joint-compatibility reasoning [9]. This defence fails exactly when "
  "the two places are not merely similar in appearance but congruent in "
  "geometry, because then the consistent transform exists.")
p("Man-made infrastructure is replete with such congruence, and engineered "
  "symmetry is its most systematic source. A four-fold symmetric jacket "
  "presents four faces that are congruent by design; a correction that rotates "
  "the estimate by ninety degrees about the structure's axis maps the "
  "structure onto itself and is therefore geometrically self-consistent while "
  "being completely wrong. To our knowledge the literature treats perceptual "
  "aliasing as a property of environments encountered at runtime, to be "
  "detected and rejected after the fact. We argue that for engineered "
  "inspection targets it is instead a property of the asset that is known "
  "before the mission, quantifiable from the drawing, and therefore available "
  "as a prior.")

h("2.6 Loosely coupled fusion and the frame-alignment problem", 2)
p("Integrating a SLAM pose into a vehicle navigation filter is usually done "
  "loosely: the SLAM system publishes a pose and an extended Kalman filter, "
  "commonly the robot_localization implementation [24], fuses it alongside "
  "velocity, attitude and depth. This requires the SLAM pose to be expressed "
  "in the filter's world frame, which it is not. The map frame is established "
  "arbitrarily at initialisation, and under a multi-map system [7] it may be "
  "re-established mid-mission. A transform between the two frames must "
  "therefore be estimated online, and the natural way to do it is to "
  "correlate the SLAM trajectory against the filter's trajectory, solving the "
  "resulting point-set registration in closed form [25], [26].")
p("The difficulty is that the filter's trajectory is partly produced by the "
  "SLAM poses the transform is used to inject. The quantity being fitted "
  "against is not independent of the fit. Residuals shrink, gates stop "
  "rejecting, and the alignment becomes self-confirming without becoming "
  "correct. We are not aware of this being identified as a systematic hazard "
  "in the underwater navigation literature, although the general principle is "
  "familiar wherever an estimator is evaluated against its own output, and we "
  "treat it in Section 4 as a property of the integration pattern rather than "
  "of any particular implementation.")

h("2.7 Evaluation practice", 2)
p("Trajectory evaluation has well-established metrics — absolute trajectory "
  "error and relative pose error as formalised with the TUM RGB-D benchmark "
  "[27], with the alignment conventions and their pitfalls set out by Zhang "
  "and Scaramuzza [28]. These assume the estimate and the ground truth are "
  "independent, which is exactly the assumption the integration pattern above "
  "violates. An evaluation can satisfy every convention in [28] and still "
  "report a number that means nothing, if the navigation solution was "
  "anchored to the reference it is being scored against.")
p("The broader reproducibility discussion in robotics [29] has concentrated "
  "on code and data availability. Pre-registration — fixing the predictions "
  "and the conditions that would falsify them before the data exist — is "
  "standard in parts of the experimental sciences [30] and rare in robotics, "
  "where simulation makes it unusually cheap. We adopt it here, and report "
  "below both the predictions that held and those that did not.")

h("2.8 The gap", 2)
p("Taken together the literature supports the following position. Underwater "
  "autonomy needs onboard navigation; dead reckoning drifts with distance in "
  "a way that white-noise simulation hides; visual SLAM can bound that drift "
  "only through loop closure; loop closure depends on place recognition; and "
  "place recognition is defeated by geometric congruence, which engineered "
  "inspection targets exhibit by design rather than by accident. The missing "
  "step is to treat that congruence as a known, quantifiable property of the "
  "asset and to arbitrate the resulting ambiguity with a sensing modality "
  "that is indifferent to appearance. Establishing whether such a scheme "
  "works additionally requires an evaluation apparatus in which the "
  "navigation estimate is not covertly anchored to the reference, which we "
  "found is not guaranteed by following standard practice.")

# ------------------------------------------------------- references
h("References", 1)
note("Recalled, not verified. Check every one before submission.")
refs = [
 "Offshore wind O&M cost and inspection-burden reference — TO BE SUPPLIED by the authors from a source you have read.",
 "Resident/autonomous subsea inspection vehicle reference — TO BE SUPPLIED.",
 "J. C. Kinsey, R. M. Eustice and L. L. Whitcomb, “A survey of underwater vehicle navigation: recent advances and new challenges,” IFAC Conf. Manoeuvring and Control of Marine Craft, 2006.",
 "L. Paull, S. Saeedi, M. Seto and H. Li, “AUV navigation and localization: a review,” IEEE J. Oceanic Engineering, vol. 39, no. 1, pp. 131–149, 2014.",
 "Teledyne RD Instruments / Nortek DVL product datasheets (scale factor and alignment specifications) — cite the specific unit modelled.",
 "R. Mur-Artal, J. M. M. Montiel and J. D. Tardós, “ORB-SLAM: a versatile and accurate monocular SLAM system,” IEEE Trans. Robotics, vol. 31, no. 5, pp. 1147–1163, 2015.",
 "C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. M. Montiel and J. D. Tardós, “ORB-SLAM3: an accurate open-source library for visual, visual–inertial and multimap SLAM,” IEEE Trans. Robotics, vol. 37, no. 6, pp. 1874–1890, 2021.",
 "S. Lowry et al., “Visual place recognition: a survey,” IEEE Trans. Robotics, vol. 32, no. 1, pp. 1–19, 2016.",
 "J. Neira and J. D. Tardós, “Data association in stochastic mapping using the joint compatibility test,” IEEE Trans. Robotics and Automation, vol. 17, no. 6, pp. 890–897, 2001.",
 "E. Rublee, V. Rabaud, K. Konolige and G. Bradski, “ORB: an efficient alternative to SIFT or SURF,” ICCV, 2011.",
 "D. Gálvez-López and J. D. Tardós, “Bags of binary words for fast place recognition in image sequences,” IEEE Trans. Robotics, vol. 28, no. 5, pp. 1188–1197, 2012.",
 "R. Mur-Artal and J. D. Tardós, “ORB-SLAM2: an open-source SLAM system for monocular, stereo and RGB-D cameras,” IEEE Trans. Robotics, vol. 33, no. 5, pp. 1255–1262, 2017.",
 "J. Engel, T. Schöps and D. Cremers, “LSD-SLAM: large-scale direct monocular SLAM,” ECCV, 2014.",
 "J. Engel, V. Koltun and D. Cremers, “Direct sparse odometry,” IEEE Trans. PAMI, vol. 40, no. 3, pp. 611–625, 2018.",
 "T. Qin, P. Li and S. Shen, “VINS-Mono: a robust and versatile monocular visual–inertial state estimator,” IEEE Trans. Robotics, vol. 34, no. 4, pp. 1004–1020, 2018.",
 "M. Kaess, H. Johannsson, R. Roberts, V. Ila, J. J. Leonard and F. Dellaert, “iSAM2: incremental smoothing and mapping using the Bayes tree,” Int. J. Robotics Research, vol. 31, no. 2, pp. 216–235, 2012.",
 "K. Köser and U. Frese, “Challenges in underwater visual navigation and SLAM,” in AI Technology for Underwater Robots, Springer, 2020.",
 "F. Hidalgo and T. Bräunl, “Review of underwater SLAM techniques,” Int. Conf. Automation, Robotics and Applications (ICARA), 2015.",
 "C. O. Ancuti, C. Ancuti, C. De Vleeschouwer and P. Bekaert, “Color balance and fusion for underwater image enhancement,” IEEE Trans. Image Processing, vol. 27, no. 1, pp. 379–393, 2018.",
 "C. Li et al., “An underwater image enhancement benchmark dataset and beyond,” IEEE Trans. Image Processing, vol. 29, pp. 4376–4389, 2020.",
 "S. Rahman, A. Quattrini Li and I. Rekleitis, “SVIn2: a multi-sensor fusion-based underwater SLAM system,” Int. J. Robotics Research, 2022 (see also IROS 2019).",
 "M. Ferrera, V. Creuze, J. Moras and P. Trouvé-Peloux, “AQUALOC: an underwater dataset for visual–inertial–pressure localization,” Int. J. Robotics Research, vol. 38, no. 14, 2019.",
 "M. Cummins and P. Newman, “FAB-MAP: probabilistic localization and mapping in the space of appearance,” Int. J. Robotics Research, vol. 27, no. 6, pp. 647–665, 2008.",
 "T. Moore and D. Stouch, “A generalized extended Kalman filter implementation for the Robot Operating System,” Intelligent Autonomous Systems 13, 2014.",
 "B. K. P. Horn, “Closed-form solution of absolute orientation using unit quaternions,” J. Optical Society of America A, vol. 4, no. 4, pp. 629–642, 1987.",
 "S. Umeyama, “Least-squares estimation of transformation parameters between two point patterns,” IEEE Trans. PAMI, vol. 13, no. 4, pp. 376–380, 1991.",
 "J. Sturm, N. Engelhard, F. Endres, W. Burgard and D. Cremers, “A benchmark for the evaluation of RGB-D SLAM systems,” IROS, 2012.",
 "Z. Zhang and D. Scaramuzza, “A tutorial on quantitative trajectory evaluation for visual(-inertial) odometry,” IROS, 2018.",
 "F. Bonsignorio and A. P. del Pobil, “Toward replicable and measurable robotics research,” IEEE Robotics and Automation Magazine, vol. 22, no. 3, 2015.",
 "B. A. Nosek, C. R. Ebersole, A. C. DeHaven and D. T. Mellor, “The preregistration revolution,” PNAS, vol. 115, no. 11, pp. 2600–2606, 2018.",
 "M. M. M. Manhães, S. A. Scherer, M. Voss, L. R. Douat and T. Rauschenbach, “UUV Simulator: a Gazebo-based package for underwater intervention and multi-robot simulation,” OCEANS, 2016.",
 "P. Cieślak, “Stonefish: an advanced open-source simulation tool designed for marine robotics, with a ROS interface,” OCEANS Marseille, 2019.",
]
for i, r in enumerate(refs, 1):
    d.add_paragraph(f"[{i}] {r}", style="List Number" if False else None)

# ---------------------------------------------------- status note
d.add_page_break()
h("Status note for the reviewer of this draft", 1)
note("Not part of the manuscript. This records what the prose above is "
     "entitled to assume as of 2026-10-14, so that nothing unsupported is "
     "carried forward silently.")

p("Settled, and safe to cite in later sections:", italic=True)
for s in (
    "R1 — jacket faces are geometrically interchangeable. Measured "
    "independently from sonar and from CAD (ICP fitness 1.000, RMSE ~0.11 m). "
    "Algorithm-free and calibration-independent.",
    "The observability defect and its magnitude. Final position error 41.04 m "
    "with the alignment fitted against the filter it feeds, 1.68 m against an "
    "independent reference, same seed and configuration.",
    "The three-seed reference-arm result, pre-registered. The SLAM-free "
    "reference filter reaches 0.11 / 0.47 / 0.57 m final error against the "
    "no-SLAM arm's 0.11 / 0.34 / 0.51 m, and the primary filter is 8–10x worse "
    "on every seed.",
    "The DVL error model is verified per run: the realised 3x3 operator is "
    "recovered from the recorded velocities and matched against the seed, to "
    "better than 0.0005.",
):
    d.add_paragraph(s, style="List Bullet")

p("Open, and NOT safe to assume in the prose:", italic=True)
for s in (
    "R3 — the 90-degree re-lock signature. The p = 0.0087 dose-response was "
    "measured on 2026-10-07, before the observability defect was found, so it "
    "comes from closed-loop runs whose alignment was later shown to be "
    "unfalsifiable. It must be re-run with the reference filter open. A "
    "precondition check on the three open-loop runs available (40 locks, "
    "baseline structure only) gives 4-fold as the strongest of six folds "
    "tested, R = 0.253, p = 0.076 — the right direction, pre-specified, but "
    "not significant, and not the dose-response.",
    "The mechanism sentence in Section 1 and Section 2.5. The draft says "
    "place recognition aliases between faces. ORB-SLAM3's own console logs for "
    "the three runs report ONE loop detection in total across three complete "
    "orbits (1, 0, 0), with 41–202 tracking losses and 2–14 new maps created "
    "per run. Whatever produces the re-locks, it is not predominantly false "
    "loop closure, because loop closure almost never executes. The mechanism "
    "must be restated to match this before the Introduction is final.",
    "R4 — the symmetry-aware gate. Designed, not yet implemented or evaluated. "
    "The Introduction currently lists it as a contribution; that must not "
    "survive into submission unless it is built and measured.",
    "Whether visual SLAM can bound the drift at all on this target. The best "
    "possible global similarity transform between the SLAM track and ground "
    "truth leaves a 16.5–16.8 m residual on a ~40 m trajectory, so at present "
    "there is no fixed map-to-world transform to recover. Work is underway on "
    "the tracking failures upstream of this.",
):
    d.add_paragraph(s, style="List Bullet")

out = "DeepSight_Introduction_LitSurvey_v1.docx"
d.save(out)
print("wrote", out)
