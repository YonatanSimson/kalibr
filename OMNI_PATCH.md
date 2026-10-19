# Omni (MEI) patch: fixed ξ, factory-form distortion, wide-FOV fixes

Changes on top of upstream kalibr (base commit `1f60227`) for calibrating very wide (> 180°)
fisheyes with the MEI model the way camera makers store it. With none of the variables below set,
existing models behave as stock (regression at the end).

## Models

| kalibr model | distortion | coefficients (`distortion_coeffs`) |
|---|---|---|
| `omni-radtan` (stock) | radtan | k1 k2 p1 p2 |
| `omni-radtan4` | `RadialTangential4Distortion`: radial to r⁸ | k1 k2 p1 p2 k3 k4 |
| `omni-radtanpro` | `RadialTangentialProDistortion`: Insta360's 13-term `offset_v6` form | d0..d12 (vendor order) |

radtan4's parameter order starts with radtan's, so a radtan result seeds it unchanged. Its
tangential terms are OpenCV-ordered (x gets 2·p1·xy + p2·(r² + 2x²)).

radtanpro is telemetry-parser's `MeiDistortion::RadtanPro`, on MEI-normalised x, y:

    radial = 1 + r²(d0 + r²(d1 + r²(d2 + r²(d3 + r²·d4))))
    a = d5 + d7·r²,   b = d6 + d8·r²
    x' = x·radial + (r² + 2x²)·a + 2xy·b + d9·r² + d11·r⁴
    y' = y·radial + (r² + 2y²)·b + 2xy·a + d10·r² + d12·r⁴

In radtan order `a` is p2 and `b` is p1. Coefficients stay in the vendor's order, so a factory
record seeds the model unchanged.

## Environment variables

| variable | effect |
|---|---|
| `KALIBR_OMNI_XI_INIT=<ξ0>` | `initializeIntrinsics` seeds ξ = ξ0 instead of 1, converting the line-fit focal to f = γ(1 + ξ0)/2 so the near-axis focal is unchanged. |
| `KALIBR_OMNI_XI_FIXED=1` | ξ leaves the optimisation: `minimalDimensions()` 5 → 4, `update()` skips ξ, the intrinsics Jacobian drops its ξ column. |
| `KALIBR_RADTANPRO_FIXED="i j …"` | radtanpro: hold coefficients d_i at their current value. They leave the parameter block (`minimalDimensions()` 13 → 13 − n, the parameter Jacobian drops their columns, `update()` skips them), as fixed ξ does. Insta360 stores d4 = 0 on every unit seen, so `"4"` mirrors the factory form. |
| `KALIBR_OMNI_DIST_INIT="d…"` | seeds the distortion in `initializeIntrinsics`, in the model's own coefficient order; the count must match or the seed is ignored with a warning. Needed with ξ > 1: undistorted MEI only lifts r² ≤ 1/(ξ² − 1), which excludes the rim of a ~190° lens. |

ξ is removed from the parameter block rather than given a zero Jacobian column: kalibr's
rank-revealing view selection would see a permanent rank deficiency, and diagonal-scaled LM
damping leaves a zero column singular. `CameraGeometry` stacks Jacobians into dynamic matrices
sized from `minimalDimensions()`, so the smaller block is consumed correctly.

Set the variables only for omni runs: DS / EUCM `initializeIntrinsics` call the omni initialiser
internally and assume ξ = 1.

`kalibr_calibrate_cameras --shuffle-seed N` makes runs repeatable. Without it kalibr shuffles the
view order unseeded, and two runs on the same data can differ by ~10 px in focal.

## Wide-FOV fixes (Omni-, DoubleSphere-, ExtendedUnifiedProjection)

1. **PnP with < 6 points.** `estimateTransformation` called `cv::solvePnP` with 4–5 points;
   OpenCV 4 throws, and the uncaught exception aborted `initializeIntrinsics` for every
   omni-initialised model on a 193° lens. Now: at least 6 points, and a `cv::Exception` means
   "no pose".
2. **Pose guess dropped rim boards.** Only corners within 80° of the optical axis went into the
   pinhole PnP, so boards near the rim got no pose (the X6 rear lens started from 1 usable view and
   diverged). Now the rays are rotated so their mean is +z, PnP runs there, and the pose is rotated
   back.
3. **Failed pose guesses poisoned the intrinsics pre-solve.** `CameraIntializers.calibrateIntrinsics`
   added views whose pose guess failed with a garbage pose (NaN). They are now skipped.

New camera geometries are registered for boost serialization (`aslam_cv_serialization`, including
`gen_files.py`); kalibr copies the detector through it and fails with "unregistered class"
otherwise.

The Docker image (`Dockerfile_omni_xi`) also installs xvfb and replaces upstream's shell-form
`ENTRYPOINT`, which ignores the command `docker run IMAGE cmd` passes; otherwise every runner call
silently ran nothing.

## Build and test

    ./build_omni_xi_image.sh [tag]      # default kalibr_ubuntu2004_omnixi, layered on kalibr_ubuntu2004

It packs every file changed since `1f60227` and runs an incremental `catkin build`. Files deleted
since the base are not removed from the image.

gtests (`aslam_cameras`):
- radtan4 parameter and point Jacobians vs finite differences, distort/undistort round trip to the
  X6 rim, and the camera-geometry harness on `Radtan4DistortedOmni`;
- radtanpro: distortion equal to the vendor formula (verbatim copy, X6 factory terms), parameter
  and point Jacobians vs finite differences, round trip to the rim, fixed terms (dimensions, Jacobian
  columns, `update()` leaves them alone), and the harness on `RadtanProDistortedOmni`;
- `test*FixedXi`: fixed ξ switched on in-process (`omni_xi::setFixed`), 4-column Jacobian vs
  finite differences and vs the free-ξ Jacobian without its ξ column, at ξ = 0.5, 1, 2, 2.5, 3
  (radtan, radtan4, radtanpro).

`test_omni_xi.sh` runs the end-to-end matrix. Pre-existing upstream failures:
testTriangulationNoisy and GridCalibration (relative image paths / random noise).

## Regression vs stock

GoPro 1080/30 wide bag (C3531325057330), no variables set, `--no-shuffle --bag-freq 4`:

| model | stock | patched | views |
|---|---|---|---|
| pinhole-equi | fu 880.1134, σ 0.3040/0.3143 | identical to ~1e-13 | 38 / 38 |
| eucm-none | fu 875.3798, σ 0.3111/0.3014 | identical to ~1e-13 | 30 / 30 |
| omni-radtan | ξ 1.230676, fu 1947.458, σ 0.344054/0.327217 | ξ 1.231326, fu 1948.027, σ 0.344054/0.327214 | 42 / 42 |
| ds-none | ξ −0.1580, fu 739.29, σ 0.3017/0.2881 | ξ −0.1549, fu 741.89, σ 0.3007/0.2881 | 27 / 28 |

omni-radtan reaches the same minimum (the pose-guess fix changes the start poses slightly, and
kalibr stops at its 1e-3 step tolerance). DS rescues one more view, with the same fit along its
ξ–α–f near-degeneracy.
