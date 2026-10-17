# Citation and novelty check, 2026-10-17

Method: web search on each reference's title, authors, venue and year. Only
titles, abstracts and bibliographic records were seen; **no paper was read.**
So this confirms a reference *exists and is stated correctly*, not that it
*supports the sentence it is cited for*. That was checked for [8], [9], [23]
only.

## The 32 references

| # | result | note |
|---|---|---|
| 1 | placeholder | no verified source for an inspection share of O&M cost; press figures range from 60 to 90 % depending on what is counted, and are not comparable. Do not quote a share. For market growth, GWEC Global Offshore Wind Report 2025 exists (8 GW added in 2024, China leading) |
| 2 | placeholder | only trade press found (Beam on jackets at Seagreen, BeeX on monopiles at Nordsee One); their navigation stacks are not described |
| 3 | confirmed | Kinsey, Eustice, Whitcomb, IFAC MCMC 2006 |
| 4 | confirmed | Paull et al., IEEE JOE 39(1):131-149, 2014 |
| 5 | placeholder | cite the specific DVL datasheet |
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
break it, and the effect is not resolved here. Still needed: an academic
source describing jacket geometry. Only project pages and patents were found.

## Not checked, and why

- **Does each cited paper support its sentence?** Not read; checked for
  [8], [9], [23] only.
- **Axis position per scenario** and **the cost in the other four wrong-face
  missions.** Need the bag recordings; the external drive ("more space") was
  not mounted, and `bags/` is a link to it.
- **Placeholders [1], [2], [5].**
