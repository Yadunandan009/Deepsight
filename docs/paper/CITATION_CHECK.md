# Citation and novelty check, 2026-10-17

Method: web search on each reference's title, authors, venue and year. Only
titles, abstracts and bibliographic records were seen; **no paper was read.**
So this confirms a reference *exists and is stated correctly*, not that it
*supports the sentence it is cited for*. That was checked for [8], [9], [23]
only.

## The 32 references

| # | result | note |
|---|---|---|
| 1 | confirmed from the supplied page (2026-10-23) | ATLANTIS, "The Atlantic Testing Platform for Maritime Robotics: New Frontiers for Inspection and Maintenance of Offshore Energy Infrastructures", EU Horizon 2020 grant 871571 (coordinator INESC TEC), CORDIS Periodic Reporting for period 2, 2021-07-01 to 2023-12-31, doi:10.3030/871571 (the DOI resolves to the project record; Crossref gives the grant record, 2020). Read on the supplied page: O&M up to 30 % of the cost of energy for offshore wind; robotic inspection can reduce O&M costs up to 50 % and down time by up to 70 %. The project's own claims, quoted as such. The page supplied is the Polish-language CORDIS view with English report text |
| 2 | confirmed (Crossref, 2026-10-23) | C. Mai, S. Pedersen, L. Hansen, K. L. Jepsen, Z. Yang, "Subsea infrastructure inspection: a review study", 2016 IEEE International Conference on Underwater System Technology: Theory and Applications (USYS), pp. 71-76, doi:10.1109/USYS.2016.7893928. Crossref record has no abstract; the 'ROV needs a support ship' content was from a web abstract earlier and is not re-read. About pipelines and cables, not jackets, so cited for the general statement only |
| 3 | confirmed | Kinsey, Eustice, Whitcomb, IFAC MCMC 2006 |
| 4 | confirmed | Paull et al., IEEE JOE 39(1):131-149, 2014 |
| 5 | confirmed, manufacturer's datasheet read (2026-10-24) | Water Linked AS, "The world's smallest DVL - DVL A50", product datasheet, 2 pp., PDF created 2026-08-06, https://waterlinked.com/datasheets/dvl-a50, accessed 2026-10-24. Read: long-term accuracy +-1.01 % (Standard version), +-0.1 % (Performance version); 1 MHz; 4-beam convex Janus; 22.5 deg beam angle; altitude 5 cm to 50 m; max velocity 3.75 m/s; resolution 0.1 mm/s. Replaces the Blueye Robotics reseller copy, which carried the same accuracy figures |
| 6 | confirmed | ORB-SLAM, T-RO 31(5):1147-1163, 2015 |
| 7 | confirmed | ORB-SLAM3, T-RO 37(6):1874-1890, 2021 |
| 8 | confirmed | Lowry et al., T-RO 32(1):1-19, 2016 |
| 9 | confirmed, **mis-cited once** | Neira & Tardos, T-RA 17(6):890-897, 2001. About data association (joint compatibility). Right for the RANSAC / joint-compatibility sentence; not a source for perceptual aliasing, so the aliasing sentence now cites [8], [23] |
| 10 | confirmed | ORB, ICCV 2011, pp. 2564-2571 |
| 11 | confirmed | Galvez-Lopez & Tardos, T-RO 28(5):1188-1197, 2012 |
| 12 | confirmed | ORB-SLAM2, T-RO 33(5):1255-1262, 2017 |
| 13 | confirmed | LSD-SLAM, ECCV 2014, LNCS 8690 pp. 834-849 |
| 14 | confirmed | DSO, TPAMI 40(3):611-625, 2018 |
| 15 | exists, detail unconfirmed | VINS-Mono, T-RO 2018. Volume 34, issue 4 and pages 1004-1020 were not confirmed by the search |
| 16 | exists, pages differ | iSAM2, IJRR 31(2), 2012. Sources give 216-235 (MIT DSpace) and 217-236 (CMU). Check the publisher |
| 17 | confirmed | Koser & Frese, Springer 2020, pp. 125-135 (online 2019) |
| 18 | **replaced** | the Hidalgo & Brauml entry could not be found (two searches). Replaced by Heshmat, Saad Saoud, Abujabal, Sultan, Elmezain, Seneviratne, Hussain, "Underwater SLAM meets deep learning: challenges, multi-sensor integration, and future directions", Sensors 25(11):3258, 2025 (DOI 10.3390/s25113258). The first author's name differs between MDPI/PMC (Heshmat) and the Khalifa University record |
| 19 | confirmed | Ancuti et al., TIP 27(1):379-393, 2018 |
| 20 | confirmed | Li et al., TIP 29:4376-4389, 2020 |
| 21 | confirmed, completed | SVIn2, IJRR 41(11-12):1022-1042, 2022 |
| 22 | confirmed, completed | AQUALOC, IJRR 38(14):1549-1559, 2019 |
| 23 | confirmed | FAB-MAP, IJRR 27(6):647-665, 2008. Its abstract addresses perceptual aliasing and visually repetitive scenes explicitly |
| 24 | confirmed | Moore & Stouch, IAS-13, 2014 |
| 25 | confirmed | Horn, JOSA A 4(4):629-642, 1987 |
| 26 | confirmed | Umeyama, TPAMI 13(4):376-380, 1991 |
| 27 | confirmed | Sturm et al., IROS 2012 |
| 28 | confirmed | Zhang & Scaramuzza, IROS 2018, pp. 7244-7251 |
| 29 | confirmed, one source | Bonsignorio & del Pobil, IEEE RAM 22(3):32-35, 2015 |
| 30 | confirmed | Nosek et al., PNAS 115(11):2600-2606, 2018 |
| 31 | confirmed | UUV Simulator, OCEANS 2016 Monterey |
| 32 | confirmed | Stonefish, OCEANS Marseille 2019 |
| 33-35 | **added** | the three robust-SLAM papers below that were fully confirmed |

Totals: 29 real references; 26 confirmed in full (including the replacement
for 18), 3 exist with a detail unconfirmed or ambiguous (15, 16, 29).
3 placeholders.

## Prior art found: what this does to the novelty claim

**Rejecting false loop closures is an established problem.**

| work | what it does | status |
|---|---|---|
| Sunderhauf & Protzel, IROS 2012, pp. 1879-1884 | switchable constraints: the optimiser rejects false loop closures | confirmed, added as [33] |
| Olson & Agarwal, IJRR 32(7), 2013 (RSS 2012) | max-mixtures: model the failure probability of each closure | confirmed, added as [34] |
| Mangelson et al., ICRA 2018, pp. 2916-2923 | pairwise consistency: largest mutually consistent set of closures | confirmed, added as [35] |
| Agarwal et al., ICRA 2013 | dynamic covariance scaling | existence confirmed; authors from memory |
| Latif, Cadena, Neira, IJRR 2013, "Robust loop closing over time" | clusters closures, chi-squared consistency | existence confirmed; details from memory |
| ROVER, arXiv 2508.13488 (2025) | **closest to ours**: verifies loop closures against the robot's own trajectory in repetitive environments | existence confirmed; authors not retrieved |
| DRACo-SLAM, arXiv 2210.00867 | notes perceptual aliasing is persistent in imaging-sonar SLAM and needs strict outlier rejection | existence confirmed; authors not retrieved |
| DARE-SLAM, arXiv 2102.05117 | uses pairwise consistency to reject closures that conflict with odometry | existence confirmed |

**Consequences.**
- "Check a loop closure against dead reckoning" is **not new**. ROVER and the
  chi-squared / consistency literature do it. Our plain size check is prior art.
- "Perceptual aliasing in underwater SLAM" is **not new**. DRACo-SLAM states it.
- What the search did **not** find: any method that uses a known
  *rotational symmetry group of the target* to recognise a wrong-face
  closure, or any quantification of wrong-face closures on a jacket-type
  structure with camera SLAM. That is where the narrowed claim lives.
- **Caveat on that claim.** This was a web search, not a systematic review.
  "Not found" is not "does not exist". Before the paper says "to our
  knowledge", search IEEE Xplore, Scopus and Google Scholar on: rotational
  symmetry, symmetry group, repetitive structure, loop closure, jacket,
  pile, pier, hull inspection.
- ROVER (Aug 2025) must be read and cited, and the paper has to say how a
  symmetry-operation test differs from a trajectory-prior test.

## Second search: is a symmetry-based test already known?

Searched 2026-10-17, web only. **Not found:** any method that rejects a loop
closure because its relative rotation matches a symmetry angle of the target.
**Found, and it matters:** symmetry ambiguity is handled in two neighbouring
fields, so the *idea* of using symmetry is not new.

| field | work | what it does |
|---|---|---|
| object SLAM | Merrill et al., CVPR 2022 (arXiv 2205.01823) | symmetry- and uncertainty-aware object SLAM |
| object SLAM | arXiv 2303.07872 | uses only the unambiguous pose parameters for data association, by symmetry type |
| object SLAM | Fu et al., arXiv 2108.01225 | multi-hypothesis handling of pose ambiguity, folded into a SLAM mixture model |
| object SLAM | arXiv 2107.09265 | max-mixture back end that keeps all symmetric hypotheses and disambiguates over time |
| global localisation | Milstein, Sanchez, Williamson, AAAI 2002 (arXiv cs/0204044) | clustered particle filter for highly symmetric maps |
| global localisation | Sehestedt & Schneider, 2006 | two-stage resampling for symmetric environments |
| loop closure | IROS 2022 structure-aware loop closing, spherical harmonics on sparse point clouds | reduces perceptual aliasing in repetitive structure (title not retrieved) |

Existence was confirmed from titles and abstracts; none were read, and author
lists and initials were not confirmed, so none are in the reference list yet.
Read and cite them in the section 2.5 rewrite.

**Consequence.** A symmetry-based loop-closure test reads as an application of
known symmetric-ambiguity handling (multi-hypothesis, max-mixtures) to
loop closures on a symmetric *structure*. That is an incremental claim, and a
reviewer will say so. The novelty has to come from elsewhere as well.

## Related to the sonar idea

Pose drift limiting acoustic reconstruction is studied ("Acoustic Neural 3D
Reconstruction Under Pose Drift", arXiv 2503.08930; Robocademy results,
arXiv 1910.13144: accuracy limited by sonar precision and platform
navigation). Not found: an evaluation against a CAD mesh of a jacket-type
structure that compares dead reckoning, SLAM-fused and a wrong-face closure.
Web search only; not exhaustive.

## The jacket premise

The draft said jackets are built four-fold symmetric "because a symmetric
truss distributes loading equally". **Not supported.** Sources found describe
jackets with three, four or more legs, and boat landings, cable tubes and
ladders as standard fittings (a Dogger Bank substation jacket lists 20 cable
tubes and a boat landing). None says symmetry is a design aim. The
Introduction now says four-legged designs with a repeating bracing pattern
are common, their primary steel is close to four-fold symmetric, appurtenances
break it, and the effect is not resolved here. Resolved 2026-10-23 with [45] (OC4 report on the UpWind reference jacket: four legs, X-braces, appurtenances omitted).

## Not checked, and why

- **Does each cited paper support its sentence?** Not read; checked for
  [8], [9], [23] only.
- **Axis position per scenario** and **the cost in the other four wrong-face
  missions.** Need the bag recordings; the external drive ("more space") was
  not mounted, and `bags/` is a link to it.
- **Placeholders [1], [2], [5].**

---

# Pass 2, 2026-10-19: does each reference support its sentence?

Method: each sentence carrying a citation was extracted from the document and
judged against what the reference is known to contain, with web lookups for the
doubtful ones. **Still not done:** reading each paper in full. The judgements on
standard references below rest on knowledge of those papers, not on re-reading
them today; only the items marked *looked up* were checked against a source today.

## New references, [36] to [42]

| # | result | note |
|---|---|---|
| 36 | looked up (arXiv abstract page) | Merrill, Guo, Zuo, Huang, Leutenegger, Peng, Ren, Huang; CVPR 2022; symmetric objects: uses the SLAM camera pose as a prior when tracking keypoints. Pages not retrieved |
| 37 | looked up | Fu, Huang, Doherty, Wang, Leonard; IROS 2021; multiple pose hypotheses fed to a mixture model, resolved over observations. Pages not retrieved |
| 38 | looked up | Lu, Huang, Doherty, Leonard; arXiv 2107.09265, 2021; max-mixtures, temporally consistent hypotheses. **No venue found; cited as arXiv.** The earlier table in this file called it a "max-mixture back end"; that is right, the author list was missing |
| 39 | looked up | Lee, Jang, Kim; ICRA 2023; three symmetry categories, uses only the unambiguous parameters for data association. Pages not retrieved |
| 40 | looked up, **earlier note corrected** | Sanchez, Milstein, Williamson; AAAI 2002; title "Robust global localization using clustered particle filtering". The first search (above) listed the authors as Milstein, Sanchez, Williamson and gave the topic correctly; the arXiv page gives Sanchez first |
| 41 | looked up (arXiv abstract and HTML page) | ROVER: Yu, Yang, Jiao, Hu, Liu, Wang, Tan, Zhang; arXiv 2508.13488, Aug 2025 (a v2 is dated Sept 2026). See below |
| 42 | **recalled, not looked up** | Fischler & Bolles, Commun. ACM 24(6):381-395, 1981. Added because [9] (joint compatibility) was the only citation for a sentence that also names RANSAC |

**ROVER, what it does** (from the HTML page; the last 14,500 characters of the
page, and the PDF, were not read; this is a read of the method section, not of the
whole paper). The trajectory prior is the front-end odometry keyframe trajectory.
For each candidate closure it runs pose-graph optimisation with the closure added,
aligns the two trajectories with a Sim(3) transform on translation only, takes the
RMS distance as a score, and accepts or rejects against a fixed threshold
(0.5 m indoors, 1.0 m outdoors in its experiments). It runs after geometric
verification in its ORB-SLAM2 integration. Its datasets are simulated rooms and
warehouses, a real escalator and cross-floor indoor sequences, and a forest.
**It does not analyse symmetry or rotational ambiguity**; the page names
"symmetrical structures" only as an example setting. Its stated limitations
include that large odometry drift can mislead the check and that a fixed threshold
is poor over long operation, which agrees with our finding that plain checks fail
once dead reckoning has drifted by metres.

**How ours differs.** ROVER compares a candidate closure's effect on the trajectory
with the odometry trajectory. Our plain checks do the same in a simpler form, so
they are prior art. The chord check does not use a trajectory: it asks whether the
closure's length equals the chord of a symmetry operation of the structure. ROVER's
method cannot see a closure that is wrong but consistent with a smooth trajectory
change; that is the case a symmetry test is for, and ROVER does not claim to handle it.

## Sentences changed, and why

| ref | sentence | problem | change |
|---|---|---|---|
| [15] | "VINS-Mono ... without loop closure they remain odometry" | VINS-Mono has loop detection and 4-DOF pose-graph optimisation (looked up) | reworded: it has both; the odometry core alone has error bounded in rate |
| [21] | "among the strongest demonstrated underwater SLAM systems" | a superlative no source in the list supports | "a representative multi-sensor underwater SLAM system" |
| [19], [20] | "enhancement improves feature yield but does not create structure" | the two papers are enhancement methods and a benchmark; neither says this | marked in the text as the authors' inference |
| [9] | "RANSAC or joint-compatibility reasoning [9]" | [9] covers only joint compatibility | RANSAC given its own citation [42] |
| [29] | "reproducibility discussion ... has concentrated on code and data availability" | abstract not found; the characterisation is unsupported | "reproducibility and measurable experimentation ... have been argued for" |
| [30] | "rare in robotics" | no citation; unsupported | "in the robotics literature we have read, uncommon" |

## Still unsupported or unconfirmed

- **[1], [2], [5]: placeholders.** The sentence "the single change with the largest
  effect on inspection economics [1], [2]" has no verified source and **must not
  be submitted as it stands**; supply a source the authors have read, or soften it.
- **[29]**: the abstract could not be found; only the record (IEEE RAM 22(3):32-35).
- **[16]**: pages are 216-235 in the MIT record, 217-236 on the authors' page
  and at CMU. Check the publisher's version.
- **[15]**: volume, issue and pages were not confirmed by today's lookup either;
  the DOI found is 10.1109/TRO.2018.2853729.
- **Every other judgement** (the ORB-SLAM family, the DVL and USBL statements,
  Kinsey and Paull on positioning, FAB-MAP, Lowry, Horn, Umeyama, the TUM
  benchmark, Zhang and Scaramuzza, robot_localization) is from knowledge of the
  papers. None was re-read.
- **The symmetry papers [36]-[40]** were read at abstract level. Whether each does
  exactly what the new section 2.5 says is as stated in the abstract, not checked
  against the full text.


## Added 2026-10-23 from the author's early-stage reading list

| # | paper | used for | what was read |
|---|---|---|---|
| 43 | confirmed from the supplied PDF (2026-10-23) | X. Wang, X. Fan, P. Shi, J. Ni, Z. Zhou, "An Overview of Key SLAM Technologies for Underwater Scenes", Remote Sensing 15(10), article 2496, 2023, doi:10.3390/rs15102496 (printed on the PDF's first page). Review. Supports: DVL, IMU, depth sensor and compass named as proprioceptive sensors |
| 44 | E. Olivastri, D. Fusaro, W. Li, S. Mosco, A. Pretto, "A Sonar-based AUV Positioning System for Underwater Environments with Low Infrastructure Density", arXiv:2405.01971, 2024 | symmetric assets give near-identical sonar observations; particle filter plus place recognition module; simulated environment | abstract and Fig. 5 text read |
| 45 | read (2026-10-23) | F. Vorpahl, W. Popko, D. Kaufer, "Description of a basic model of the 'UpWind reference jacket' for code comparison in the OC4 project under IEA Wind Annex 30", Fraunhofer IWES technical report, 19 July 2013. Read from the PDF: "four legged jacket" with four levels of X-braces, mud braces and four central piles, designed by Rambøll for the UpWind 5 MW baseline turbine at 50 m water depth; the model omits boat landings, J-tubes, anodes, cables and ladders. Supports the jacket-type description and the appurtenance sentence; does not say that jackets are symmetric by design |

Read, not used: Kazerouni et al., "A survey of state-of-the-art on visual SLAM", Expert Systems With Applications 205 (2022) 117734; Macario Barros et al., "A Comprehensive Survey of Visual SLAM Algorithms", Robotics (CEA); Xanthidis et al., "Towards Mapping of Underwater Structures by a Team of AUVs" (ISRR 2022). Searched for symmetry, aliasing, jacket, DVL: no statement found in the two visual SLAM surveys or Xanthidis et al. Not relevant to this paper: IJEET 3(2) 2012 PID and LQR controller comparison; Modasshir and Rekleitis ICRA 2020 coral reef monitoring; ODYSSEE oyster detection (arXiv:2409.07003).

## Candidate related work found by web search, 2026-10-23 (titles and snippets only; NOT read, NOT cited in the manuscript)

| paper | why it may belong in Section 2 | check before citing |
|---|---|---|
| Rohou et al., "Proving the existence of loops in robot trajectories", arXiv:1712.01341 | verifies loops with proprioceptive measurements only; close to our revisit check; notes false detections on homogeneous sea floors | authors, venue, what is actually proved |
| Ma, Ye et al., "Efficient bathymetric SLAM with invalid loop closure identification" (Lisbon ISR) | invalid loop closures as routine in underwater SLAM; particle-filter back end | venue, year, authors |
| "Visual Loop Detection for underwater Graph-SLAM", MDPI J. Mar. Sci. Eng. 10 (2022) 511 | filters wrong loops with network, matcher and geometric check | authors, claims |
| Negre Carrasco, Bonin-Font, Oliver Codina, "Cluster-Based Loop Closing Detection for Underwater SLAM in Feature-Poor Regions", ICRA 2016 | visual loop closing in feature-poor regions | pages, claims |
| arXiv:2508.13488, loop closure verification in repetitive environments | checks loops against trajectory history | title, authors |


## Final abstract check, 2026-10-24

Every reference cited in the manuscript was checked once more against its abstract (or, where the abstract is not public, an abstract-level description), for whether it supports the sentence that cites it.

| # | supports its sentence? | note |
|---|---|---|
| 3 | yes | survey of underwater navigation; acoustic positioning as a traditional aid |
| 4 | partly | abstract: past approaches used installed beacons or periodic surfacing. Kept for the acoustic-positioning sentence only; removed from the DVL error-source sentence |
| 7 | yes | abstract: "when it gets lost, it starts a new map that will be seamlessly merged with previous maps when revisiting them" |
| 8 | yes | introduction names perceptual aliasing (places that look alike) as a core difficulty |
| 9 | yes (secondary descriptions) | joint compatibility rejects spurious pairings; abstract not public |
| 11 | yes | bags of binary words with geometric verification |
| 17 | partly | challenges chapter; abstract names underwater visual navigation challenges but not the specific list. The sentence's list was changed to the terms the abstracts use |
| 18 | yes after rewording | abstract: poor visibility, dynamic lighting, sensor noise, water-induced distortion |
| 21 | yes | sonar, visual, inertial and depth fusion |
| 23 | yes | perceptual aliasing in appearance-based mapping |
| 24 | yes | 15-dimensional state, arbitrary number of sensors (package documentation; paper on request) |
| 27, 28 | yes after rewording | alignment to a reference and ATE/RPE; neither says "independent" in so many words, so the sentence now says what they do |
| 30 | yes | define questions and analysis before seeing the outcome |
| 32 | yes | Stonefish library and ROS interface |
| 33 | yes | rejects false-positive loop closures during optimisation |
| 34 | yes | max-mixtures; one erroneous closure can make the map fail catastrophically |
| 35 | yes | largest pairwise-consistent set of inter-map loop closures |
| 36 | yes | uses the SLAM camera pose as a prior for keypoints on symmetric objects |
| 37 | yes | multiple 6D pose hypotheses as a mixture |
| 38 | yes | keeps multiple hypotheses via max-mixtures; arXiv only |
| 39 | yes | keeps only unambiguous pose parameters |
| 40 | yes | clustered particle filter keeps equally probable poses in symmetric environments; author order Sanchez, Milstein, Williamson confirmed |
| 41 | yes | trajectory prior verifies loop candidates in repetitive environments; Section V-A of the full text says substantial odometry errors can mislead it. Now listed as accepted to IEEE T-ASE 2026 |
| 42 | standard | RANSAC |
| 46 | new | Tang et al., Sensors 13(11):14583-14600, 2013, doi:10.3390/s131114583. Names DVL scale factor and INS-DVL misalignment as key factors limiting INS/DVL accuracy; scale factors about 0.1 to 0.7 % on four systems |
| 47 | new | Troni and Whitcomb, J. Field Robotics 32(5):655-674, 2015, doi:10.1002/rob.21551 (Crossref). In situ alignment calibration of Doppler and attitude sensors. Supports "misalignment is real and calibrated"; does not give a typical size, so 0.5 degree stays an assumption |

Not cited, to be dropped at formatting: [6], [10], [12]-[16], [19], [20], [22], [25], [26], [29], [31].
