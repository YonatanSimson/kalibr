# Omni (MEI) with a configurable / fixed ξ — plan and status

Goal: calibrate the Insta360 X6 lenses with kalibr's `omni-radtan` (MEI) model while ξ is held at
the factory value (X6: 2.45543, X4/X5: 2.0), so fx, fy, cx, cy can be compared with the factory
record term by term. Stock kalibr always seeds ξ = 1 and always optimises it.

## Summary (2026-10-07, Insta360 X6 sn BXEA3ABHFQ7YSX, 3008×3008 @ 24 fps)

We calibrated both lenses with DS, EUCM and MEI, ended on **MEI with the distortion extended to
k1..k4** (`omni-radtan4`, the form Insta360 stores per unit), and added an option to **fix ξ** —
which is what made the MEI parameters meaningful.

- **DS — ruled out.** Its free optimum on this lens needs α < 0, outside the legal range, so legal
  DS stalls at ±3 px and fails at the rim (23 px median beyond 90°).
- **EUCM — good compact alternative.** 6 parameters, inner-field median ≈ 1.0 px (cam0), and
  stable without any fixing: α, β ≈ (0.67, 0.78) in every solve, both lenses and the joint solve.
  It has no ξ and no ξ-style degeneracy, so the fixed-ξ option does not apply to it. Its numbers
  are not comparable to the factory record term by term (different parameterisation), only by
  projection.
- **MEI — chosen.** Insta360's own model; the factory record (`offset_v6`) is accurate to ≈ 1.1 px
  median on our corners, and a k1..k4 refit in that form lands within 0.2 % focal / 0.5 px
  principal point of it.
- **Fixed ξ made the difference — for MEI.** Free ξ trades against focal length and the k-terms:
  kalibr drifted to ξ 2.1–2.3 (k1, k2) or 2.5 (k1..k4) at equal fit, with fx moving 10 % and k3/k4
  swinging (k4 up to +83). Fixed at the factory 2.45543 the parameters line up with the factory's.
- **k1..k4 vs k1, k2 (ξ fixed) — not yet established.** k1, k2 now match the factory in sign and size
  (k1, k2-only gave k2 = +0.28 vs factory −0.99). A first run showed 12–15 % lower kalibr error
  (cam0 ±0.96/0.87 → ±0.82/0.78 px), but kalibr shuffles the view order every run and a second run
  reversed cam0 (±0.89 vs ±0.92 px): the gap is within run-to-run noise. Redo with `--no-shuffle`.
  k3, k4 are not determined either (cam1 k3 8.2 / k4 −7.0 vs factory 3.1 / 8.9; no rim data).
- **What kalibr needed** (all below): seeding ξ and the distortion (MEI with ξ > 1 cannot lift rim
  pixels otherwise), three wide-FOV fixes (PnP with < 6 points, pose guess dropping rim boards,
  failed pose guesses turning the intrinsics pre-solve NaN), and serialization registration for the
  new geometry. Patched vs stock (GoPro bag, fixed view order): pinhole-equi and EUCM bit-identical,
  omni-radtan and DS the same fit (differences only from the wide-FOV pose-guess fix); same unit-test
  results; the new ξ / distortion options are inert when unset.
- **Open:** the recording has almost no board views past 90° (14 / 19 rim corners vs ≈ 21,600 /
  4,000 inside). A take with the board held at each lens's edge is needed to fix k3, k4 and to rank
  the models at the rim.

Results and the factory comparison: `~/data/calibration/insta360/BXEA3ABHFQ7YSX/3008_24/`
(`factory_comparison.md`), produced by exovision-research `map_3d/calibration/run_insta360_kalibr.sh`
and `insta360_compare_factory.py`.

## Change (fixed / seeded ξ, OmniProjection.hpp)

Environment variables read by `OmniProjection` (`KALIBR_OMNI_DIST_INIT` is under fix 3 below):

| variable | effect |
|---|---|
| `KALIBR_OMNI_XI_INIT=<ξ0>` | `initializeIntrinsics` seeds ξ = ξ0 instead of 1. The line-fit focal estimate γ is made with ξ = 1, where the near-axis focal is γ/2; it is converted to f = γ(1+ξ0)/2 so the seed keeps the same near-axis focal. |
| `KALIBR_OMNI_XI_FIXED=1` | ξ leaves the optimisation: `minimalDimensions()` 5 → 4, `update()` skips ξ, the intrinsics Jacobian drops its ξ column. ξ stays at whatever it was set to (the seed, or a camchain value). |

Why drop the parameter rather than zero its Jacobian column: kalibr's incremental estimator
uses a rank-revealing solve to decide which views add information; a dead column is a
permanent rank deficiency, and the LM damping is diagonal-scaled, so a zero column stays
singular. Removing the dimension keeps the problem well-posed. The Jacobians are stacked into
dynamic `Eigen::MatrixXd` by `CameraGeometry`, sized from `minimalDimensions()`, so a 4-column
projection block is consumed correctly.

Caveat: `DoubleSphereProjection` / `ExtendedUnifiedProjection::initializeIntrinsics` call the
omni initialiser internally and assume ξ = 1. The variables must be set only for omni runs
(the Insta360 runner does that per kalibr invocation).

## Bug fix found on the way: PnP with < 6 points aborts the calibration

Stock kalibr fails on the X6 (193° FOV) for **every** omni-initialised model (omni, ds, eucm):

    RuntimeError: OpenCV(4.2.0) calibration.cpp:1171: cvFindExtrinsicCameraParams2
    DLT algorithm needs at least 6 points ... 'count' is 5

`estimateTransformation` (Omni-, DoubleSphere-, ExtendedUnifiedProjection.hpp) back-projects the
corners, keeps those within 80° of the axis, and calls `cv::solvePnP` if at least **4** remain.
OpenCV 4.x needs 6 there and throws; nothing catches it, so one bad candidate view during the
focal-length line search aborts `initializeIntrinsics`. Fix (all three headers): require 6 points
and treat a `cv::Exception` from `solvePnP` as "pose not estimated" (return false), which is what
every caller already handles.

## Bug fix 2: pose guess discards boards near the rim

`estimateTransformation` (same three headers) also keeps only corners whose back-projected ray
is within 80° of the **optical axis** before the pinhole PnP. On a 193° lens a board seen near
the rim has few or no such corners, so its pose guess fails. On the X6 rear lens (cam1) almost
every view failed, kalibr started the optimisation with 1 usable view and diverged to NaN.
Fix: keep every corner that back-projects, rotate the rays so their mean is +z (a virtual pinhole
looking at the board), keep rays within 80° of that, solve PnP there and rotate the pose back.
Same pose for central boards; rim boards now get one too.

## Fix 3: omni with ξ > 1 needs its distortion seeded too

Omni ξ = 2.45543 (fixed or seeded) gave NaN intrinsics on both lenses. Undistorted MEI only lifts a
normalised radius r² ≤ 1/(ξ² − 1): r ≤ 0.446 at ξ = 2.455, while the X6 rim sits at r ≈ 0.52. The
factory reaches the rim through its radial terms (k1 = 1.30, ...); kalibr starts distortion at 0,
so no rim corner lifts. Stock kalibr never hits this because it seeds ξ = 1 (no limit).
- `KALIBR_OMNI_DIST_INIT="k1 k2 p1 p2"`: `initializeIntrinsics` also seeds the distortion
  (any distortion model; the count must match its parameter vector).
- `CameraIntializers.calibrateIntrinsics` skipped nothing when a view's pose guess failed and added
  it with a garbage pose → NaN. Failed views are now skipped.
Runner: `--omni-dist "1.30284 -0.99313 0 0"` (X6 factory k1, k2).

## Build

`./build_omni_xi_image.sh [tag]` (default `kalibr_ubuntu2004_omnixi`), layered on `kalibr_ubuntu2004`:
packs every file changed since the upstream base commit and runs an incremental `catkin build`.
Files deleted since the base are not removed from the image.

## Test plan

1. [x] Patch compiles (incremental catkin build in the layered image).
0. [x] Stock image on the X6 take: ds-none and eucm-none both abort in initializeIntrinsics (bug above).
1a. [x] Image v2 (`kalibr_ubuntu2004_omnixi2`, bug fix 2): DS cam1 initialises (failed to NaN on v1).
1b. [x] DS/EUCM initialise and converge on both lenses (DS cam0 legal only on v1, see log).
2. [x] Unit tests: `aslam_cameras` gtest (`CameraGeometryTestHarness` checks analytic vs
       finite-difference intrinsics Jacobians) — run with the variables unset (must be
       unchanged); `test*FixedXi` switch fixed ξ on in-process (`omni_xi::setFixed`) and check
       the 4-column Jacobian against finite differences at ξ = 0.5, 1, 2, 2.5, 3 (radtan, radtan4).
3. [x] Regression: with the variables unset the patch only changes views where stock would have
       thrown. Stock cannot run this take at all, so compare stock vs patched on a take stock can
       calibrate. Run both with `--no-shuffle` — kalibr otherwise randomises the view order every
       run, and run-to-run differences swamp any code effect. Result: see the status log entry
       "legacy regression (review build)".
4. [x] Seed only: `KALIBR_OMNI_XI_INIT=2.45543` — converges; compare with the ξ = 1 seed.
5. [x] Fixed: `KALIBR_OMNI_XI_INIT=2.45543 KALIBR_OMNI_XI_FIXED=1` — output camchain has
       ξ = 2.45543 exactly; reprojection error close to the free-ξ run.
6. [-] ξ = 2.0 (X4/X5 convention): dropped for the X6 (user, 2026-10-07); `test_omni_xi.sh fixed2` keeps it for X4/X5.
7. [x] Factory comparison: fx, fy, cx, cy vs the X6 `offset_v6` record
       (`insta360_compare_factory.py`). kalibr's radtan has k1, k2 only (factory uses k1..k4),
       so k-terms will not match term by term; focal/principal point should be close.

## Stretch: factory-form MEI distortion (only if fixed ξ works)

kalibr's `radtan` has k1, k2, p1, p2; the Insta360 factory model is radial k1..k4 (to r⁸) + p1, p2
(+ further terms on X5/X6). To fit the factory form term by term inside kalibr:

- new distortion class (e.g. `RadialTangential4Distortion`: k1..k4, p1, p2) next to
  `RadialTangentialDistortion` — distort / undistort (iterative) / Jacobians wrt point and params;
- instantiate `OmniProjection<RadialTangential4Distortion>` + camera geometry typedef, python
  bindings (aslam_cv_python), and register `radtan4` in `kalibr_common/ConfigReader.py`
  (`omni-radtan4`), plus camchain read/write;
- test as above: Jacobians vs finite differences, then ξ fixed at the factory value and compare
  all of fx, fy, cx, cy, k1..k4, p1, p2 with `offset_v6`.

Status: **started 2026-10-07** (ξ fix worked; user go-ahead). Wiring, modelled on `RadialTangentialDistortion`:

| layer | file(s) |
|---|---|
| distortion class `RadialTangential4Distortion` (k1..k4, p1, p2; distort + Jacobians wrt point and params, iterative undistort, serialization) | `aslam_cameras/include/aslam/cameras/RadialTangential4Distortion.hpp`, `implementation/…`, `src/…`, `CMakeLists.txt` |
| geometry typedefs + factory strings (`Radtan4DistortedOmni`, `…Rs`) | `aslam/cameras.hpp`, `src/CameraGeometryBase.cpp` |
| python bindings | `aslam_cv_python/src/CameraProjections.cpp` (+ geometry/frame exports) |
| backend errors / design variables / camera model | `aslam_cv_backend_python/src/module.cpp`, `python/aslam_cv_backend/__init__.py` |
| kalibr model name `omni-radtan4` | `kalibr_common/ConfigReader.py` (read, validate, print, write), `kalibr_camera_calibration/CameraUtils.py` |
| tests | `aslam_cameras/test/RadialTangential4Distortion.cpp` (Jacobians vs finite differences, distort/undistort round trip), harness on the new omni geometry |

`KALIBR_OMNI_DIST_INIT` already seeds any distortion whose parameter count matches (6 values for radtan4).

Meanwhile the term-by-term comparison is covered outside kalibr by the fixed-ξ MEI refit
(k1..k4, p1, p2) in exovision-research `map_3d/calibration/insta360_compare_factory.py`.

## Status log

- 2026-10-07: plan written; DS/EUCM baseline run in progress on the stock image.
- 2026-10-07: patch applied to `OmniProjection.hpp` (`omni_xi::fixed()`, `omni_xi::init()`;
  Jacobian built into a local 2x5 then copied out as 2x4/2x5; `update`, `minimalDimensions`,
  seed conversion). Both variables unset ⇒ code path identical to stock (seed ξ = 1, f = γ).
  `Dockerfile_omni_xi` added; first image build OK (36/36 packages).
- 2026-10-07: baseline DS/EUCM on the stock image failed (PnP bug above). Guard added to the three
  projection headers; Dockerfile copies all three; image rebuilding. DS/EUCM baseline will be
  rerun on the patched image (variables unset).
- 2026-10-07: patched image rebuilt (36/36 OK). Queue on the X6 take: DS + EUCM (intrinsics, cam-IMU per
  lens, joint) → omni ξ fixed 2.45543 → omni ξ seeded 2.45543 (both intrinsics only).
- 2026-10-07: v1 image: DS cam0 converged (ξ −0.025, α 0.596, fu 814.0, fv 818.7; ±3 px); cam-IMU
  cam0 OK (R ≈ axis-aligned within 1.1°, |t| 32 mm, shift −3.0 ms). DS cam1 → NaN, 1/80 views used:
  bug fix 2 above. Building `kalibr_ubuntu2004_omnixi2`; queued v1 runs left as they are.
- 2026-10-07: v1 image, EUCM cam0 diverged (α → −65526). v2 image built (36/36 OK) and tagged as
  both `kalibr_ubuntu2004_omnixi2` and `kalibr_ubuntu2004_omnixi` (runner default). v1 results kept
  as `ds_v1img/`, `eucm_v1img/`; everything reruns on v2. DS cam1 test on v2 running (`ds_v2/`).
- 2026-10-07: v2 queue done. DS: cam0 legal only on v1 (±3 px; v2 → α < 0, MEI-like ξ 2.57, ±1.1 px),
  cam1 ±4 px, joint OK (baseline 47.7 mm, 1.25° from factory). EUCM: cam0 ±0.95 px, cam1 ±1.1 px,
  joint OK, near-axis focal 819.1 vs factory 818.7. Omni ξ 2.45543 fixed/seeded → NaN: fix 3 above.
  Dockerfile context is now the repo root (4 files copied). Building `kalibr_ubuntu2004_omnixi3`.
- 2026-10-07: v3 image (fix 3). Omni with ξ 2.45543 + factory k1,k2 seed (X6, cam0 / cam1):
  - ξ fixed:  fu 2837.6 / 2828.3, cu 1501.4 / 1490.7, k1 1.17, k2 +0.28 / +0.54; ±0.96 / ±1.28 px.
  - ξ seeded, free: ξ → 2.116 / 2.260, fu 2542 / 2657, ±0.85 / ±1.07 px (slightly better fit).
  - Corner median (held-out frames, independent detector), cam0 / cam1: factory 1.11 / 1.26, omni fixed
    1.11 / 1.18, omni free 1.09 / 1.10, EUCM 1.06 / 1.58, DS 2.26 / 2.99, our k1..k4 MEI refit 0.93 / 1.14.
  Conclusion: with k1, k2 only, ξ trades against focal and k (free ξ drifts to ~2.1-2.3 at equal fit);
  fixing ξ at the factory value is what makes kalibr's numbers comparable to the factory. Our k1..k4
  refit keeps ξ at 2.4556 when free → the k3, k4 stretch goal is what lets kalibr match term by term.
  Still open: test 2 (gtest Jacobians with ξ fixed) and test 3 (stock-vs-patched regression on a take
  stock can calibrate).
- v3 image retagged as `kalibr_ubuntu2004_omnixi` (runner default).
- 2026-10-07: tests. New gtest `testDistortedOmniFixedXi` (aslam_cameras/test/OmniCameraGeometry.cpp; runs only
  with KALIBR_OMNI_XI_FIXED=1): minimalDimensions 4, analytic 2x4 Jacobian == finite difference wrt
  [fu fv cu cv], update() leaves xi alone — PASS. Jacobian copy rewritten with dynamic blocks (the
  fixed-size 2x5 test types would not compile with `rightCols<4>`). Full suite, variables unset:
  stock 18/22, patched 19/23 (the extra one is the new test) — the same 4 fail on both
  (testTriangulationNoisy, GridCalibration x3: relative image paths / random noise; 3 fail when run
  from aslam_cameras/test). No regression.
- 2026-10-07: regression, GoPro 1080/30 wide (C3531325057330, stock can calibrate it), variables unset,
  stock `kalibr_ubuntu2004` vs patched v3 (reprojection σ x/y px):
  DS 0.40/0.38 → 0.36/0.31 (different minimum: ξ −0.28→−0.09, fu 628→796 — DS ξ/α/f are near-degenerate),
  EUCM 0.38/0.36 → 0.37/0.33 (≈ same params, cu +4 px), omni-radtan 0.37/0.32 → 0.39/0.33 (≈ identical:
  ξ 1.228/1.223, fu 1944.8/1945.0). Not bit-identical — fix 2 changes the pose guesses, and kalibr's
  incremental view selection follows them — but no loss of fit. (Superseded: these runs were shuffled,
  see "legacy regression (review build)" below; outputs deleted.)
- 2026-10-07: stretch goal started — `RadialTangential4Distortion` (header, implementation, src, CMake,
  typedefs `Radtan4DistortedOmni[Rs]CameraGeometry`, factory strings) + gtests written; bindings next.
- 2026-10-07: radtan4 wired end to end: python bindings (aslam_cv: distortion, projection, geometries,
  frames; backend: reprojection errors, design variables, `Radtan4DistortedOmni` camera model), kalibr
  (`omni-radtan4` in kalibr_calibrate_cameras, ConfigReader `radtan4` = 6 coeffs [k1 k2 p1 p2 k3 k4],
  CameraUtils output maps, omni intrinsics pre-pass also for radtan4). Image build now
  `./build_omni_xi_image.sh [tag]`: layers every file changed since 1f60227 (no per-file Dockerfile edits).
  Insta360 runner: model `omni4`; `--omni-dist` takes 6 values for it. Building `kalibr_ubuntu2004_omnixi4`.
- 2026-10-07: v4 image (`kalibr_ubuntu2004_omnixi4`, 21 changed files) builds 36/36; python import OK.
  radtan4 gtests: parameter Jacobian, point Jacobian, distort/undistort round trip to r 0.45 with X6
  factory terms, harness on Radtan4DistortedOmni — all PASS (parameter-Jacobian check uses abs+rel
  tolerance: near the axis the r^6/r^8 columns are below finite-difference resolution). Suite from
  aslam_cameras/test: 25 pass, the 2 pre-existing GridCalibration failures remain.
- 2026-10-07: first omni4 X6 run failed in corner extraction ("unregistered class - derived class not
  registered or exported"): kalibr copies the detector via boost serialization, and every geometry needs a
  BOOST_CLASS_EXPORT. Added `Radtan4DistortedOmni[Rs]CameraGeometry` to aslam_cv_serialization
  (`CameraBaseSerialization.hpp` keys, `src/autogen/Camera-*.cpp`, `autogen_cameras.cmake`, and the
  `gen_files.py` list so a regeneration keeps them). Note: FovDistortedOmni is not registered upstream either.
- 2026-10-07: review fixes (commit on top). Fixed-ξ gtests no longer need the env var: `omni_xi::setFixed`
  overrides it in-process, and `testDistortedOmniFixedXi` / `testRadtan4DistortedOmniFixedXi` run at
  ξ = 0.5, 1, 2, 2.5, 3 (Jacobian vs finite difference and vs the free-ξ Jacobian minus its ξ column,
  update() leaves ξ alone). Test points come from lifted pixels: `createRandomKeypoint` divides by
  ξ² − 1 and never returns at ξ = 1.
- 2026-10-07: **legacy regression (review build).** Image `kalibr_ubuntu2004_omnixi5` (this branch incl.
  the review fixes) vs stock `kalibr_ubuntu2004`, GoPro 1080/30 wide bag (C3531325057330), no
  KALIBR_OMNI_* variables, `--no-shuffle --bag-freq 4`, same focal guess (881) on stdin:

  | model | stock | review build | views |
  |---|---|---|---|
  | pinhole-equi (GoPro fisheye) | fu 880.1134, cu 958.7368, σ 0.3040/0.3143 | identical to ~1e-13 (rounding) | 38 / 38 |
  | eucm-none | α 0.54711, β 1.00914, fu 875.3798, σ 0.3111/0.3014 | identical to ~1e-13 | 30 / 30 |
  | omni-radtan | ξ 1.230676, fu 1947.458, cu 952.5448, σ 0.344054/0.327217 | ξ 1.231326, fu 1948.027, cu 952.5447, σ 0.344054/0.327214 | 42 / 42 |
  | ds-none | ξ −0.1580, α 0.52892, fu 739.29, σ 0.3017/0.2881 | ξ −0.1549, α 0.52938, fu 741.89, σ 0.3007/0.2881 | 27 / 28 |

  pinhole-equi and EUCM do not see any change (the only shared change, skipping views without a pose
  guess in `calibrateIntrinsics`, is a no-op when no view fails). omni-radtan lands in the same minimum
  (same views, same error) with ξ/fu 0.05 % apart — the pose-guess fix gives slightly different start
  poses and kalibr stops at its 1e-3 step tolerance. DS uses one more view (the pose-guess fix rescues
  it), same fit; fu moves along DS's ξ–α–f near-degeneracy. Fixed-ξ / init / distortion-seed options are
  inert when unset. Also on the review build: unit tests 26 pass + the 2 pre-existing GridCalibration
  failures; both fixed-ξ gtests pass; X6 legacy omni (ξ seed 1, free) converges, ξ 1.82 / 1.93,
  σ 0.84 / 1.17 px. Outputs: session scratchpad `gopro_regression/{stock,reviewed}/` and
  `~/data/calibration/insta360/review_rerun/`.
- 2026-10-07: **kalibr is non-deterministic by default**: `kalibr_calibrate_cameras` shuffles the view order
  (`random.shuffle`, unseeded) unless `--no-shuffle` is given. Two shuffled X6 runs on the same image
  differ by up to ~10 px in focal and flip DS between minima. Consequence: the earlier "k1..k4 vs
  k1, k2: 12–15 % lower error" (shuffled runs) is within run-to-run noise and is not established —
  redo that comparison with `--no-shuffle`.
