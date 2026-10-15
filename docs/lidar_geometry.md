# LiDAR geometry — the left/right fan-half offset

> **Status:** *open.* A 360° sweep assembled the normal way shows a **systematic
> ≈2.4° relative yaw between the two in-plane radial halves** of every fan
> (stable across scans: 2.05–2.51°). Everything checked so far attributes it to
> **beam indexing between the two halves**, not to a mount aim error and not to
> beam-plane (out-of-plane) geometry. This document records the evidence, the
> models that were **ruled out**, and the assumptions that remain.

## Background: why a fan has two halves

The Waveshare STL27L is a **horizontal-plane 2D LiDAR**: a ToF laser diode spins
about the module's local up-axis and reports `(range, in-plane angle)` samples of
a ~360° arc. PiLiDAR mounts it **tipped 90° to its side**, so the module's
rotation plane becomes the cloud's **vertical X–Z plane** and the module's
**spindle axis becomes cloud Y** (the disc normal).

Consequences of the 90° mount (all verified against the code):

- `merge_2D_points` maps the reported 2D fan `(x, y) = (−r·cosθ, r·sinθ)` to
  `(X, Y, Z) = (−r·cosθ, 0, r·sinθ)` and revolves it about `+Z`.
- Each fan spans the **full** 360° (≈2184 samples/plane) and splits by the
  **in-plane radial coordinate** into two halves: `x < 0` ("left") and `x ≥ 0`
  ("right"). Each half therefore also sweeps a **complete 360°**.
- The two halves see the **same surfaces**, so with a correct assembly they must
  **coincide**. Any residual between them is a mount/geometry error — this is the
  basis of `experiments/split_fans.py`.

Because each half sweeps a full 360°, a sweep wider than 180° effectively
contains **two overlapping panoramas** of the scene. They are rendered
**separately** — `<scan_id>_lidar_left.jpg` / `_lidar_right.jpg` (plus the merged
legacy `<scan_id>_lidar.jpg`) — because mixing them shows spurious brightness
steps wherever one half stands closer to the surface than the other.

## The measurement

`experiments/split_fans.py <scan_id>` assembles the halves and runs ICP:

```
261015-1726: left vs right ICP: fitness=0.994 rmse=24.2mm
  rotation 2.34 deg about axis=(-0.01,+0.01,+1.00)  translation=(-0.6,-0.5,+5.7)mm
```

The relative rotation is **about Z** (the scanner rotation axis) and the
translation is ≈0. It is **stable across scans** (261014-1954 → 2.51°,
261015-0821 → 2.05°, 261015-1213 → 2.33°, 261015-1726 → 2.34°).

Resolving it by **fan-elevation θ** (a point's elevation in the fan plane,
`η = atan2(z, √(x²+y²))`) gives a **flat** −2.4 … −3.3° across the whole populated
range `θ ∈ [−60°, +60°]`:

| θ bin | rel. yaw | `|yaw|/tanθ` |
|---|---|---|
| −60…−40° | −3.31° | 2.78 |
| −40…−20° | −2.78° | 4.81 |
| −20… 0° | −2.51° | 14.24 |
| 0…+20° | −2.37° | 13.42 |
| +20…+40° | −2.48° | 4.30 |
| +40…+60° | −2.66° | 2.23 |

The ratio `|yaw|/tanθ` is **not** constant, so the offset is **not** `∝ tanθ`.

## Models ruled out

| Hypothesis | Expected signature | Measured | Verdict |
|---|---|---|---|
| **Parallax / stereo baseline** (~2× the mount's Y offset) | shift **∝ range** | ≈0 shift, constant rotation | ✗ |
| **Mount position** wrong | shift **∝ range** | ≈0 shift | ✗ |
| **Mount aim** (pitch/roll/yaw) | changes the *relative* yaw | z: **no** effect (rotates the whole cloud); x: only ~0.1°/deg | ✗ |
| **Fan plane not through the disc centre** (aperture offset) | yaw **∝ tanθ** (0 at horizon, blowing up at the poles) | flat, not `tanθ` | ✗ |
| **Out-of-plane diode/plane tilt** (the beam traces a cone) | azimuth effect **≈0** (discarded by the flattening reconstruction) | — | ✗ (invisible) |

### Why a mount rotation cannot fix it

The mount rotation is applied to each fan **before** the revolve about Z. A z
(mount) rotation and the revolve are both about Z, so they commute and collapse
to a single constant azimuth applied to **every** plane — hence the whole cloud
turns together and the *relative* half/half offset is unchanged. Measured
directly: `ROTATION[2] = 2.34°` moved the whole cloud by −2.33° and left the
relative yaw at −2.47°. **No value in `3D.CALIBRATION.ROTATION` can remove it.**

### Why out-of-plane geometry cannot do it

The reconstruction discards the out-of-plane component: the STL27L reports only
the *in-plane* angle and the range, and `merge_2D_points` flattens every beam
into the vertical plane. A **static** out-of-plane tilt (a fixed diode tilt, or a
warped emitted "plane") therefore barely rotates the reconstruction at all — it
only re-scales azimuth by `cos γ` (0.002% at γ=1°) and produces a *range-
proportional* projection artifact. The saved simulator
`experiments/beam_model_sim.py` raycasts a synthetic room under each model:

| model | rel-yaw at γ = 1–3° | at the horizon (θ≈0) | matches −2.4° flat? |
|---|---|---|---|
| fixed diode tilt (`cone`) | +0.1 … −0.5° | ≈ **0** | ✗ |
| whole emitted plane tilted (`tilt`, δ ∝ sinθ) | ~+0.2° | ~0 | ✗ |
| perfect plane | −0.15° | ≈0 | ✗ |

None reproduces a flat −2.4°.

## What it actually is

A **constant relative azimuth (angular-index) offset between the two halves**:

1. It is **constant in θ** (rules out fan-centering/aperture geometry).
2. It is **≈0 lateral shift** and **does not grow with range** (rules out
   parallax and mount *position*).
3. It is **not removable by any mount aim** (rules out mount rotation — see above).
4. It is **stable across scans** → a property of the instrument, not the setup.

Each half is internally consistent (its self-residual is at the noise floor — one
clean rigid offset, not a fan), and the two halves are mutually indexed ≈2.4°
apart. This points at the **reset/seam between the two radial sectors** of the
LiDAR's own scan (the angular indexing the firmware assigns across the 360°),
not at the beam-plane geometry.

A global *in-plane* angular bias (encoder / code-wheel) is a candidate mechanism,
**but a purely global one self-cancels** for a 360° split (both halves see the
same angles). What fits is a defect **gated to one half** — i.e. the angular
indexing is not symmetric between the `x>0` and `x<0` sectors. A once-per-rev
(loop-closure / eccentricity) harmonic was also tested on the real per-azimuth
data: the coherent sinusoidal amplitude is only **0.6°** half-to-half
(`A=0.30°`), far short of 2.4°.

## Assumptions

- The assembly (`merge_2D_points`) faithfully represents the geometry: reported
  in-plane angle ⇒ cloud X–Z, range preserved, out-of-plane discarded.
- The mount is a proper rigid transform. `ROTATION` was upgraded to a **full
  euler-xyz** orientation in v2.6.23 (before that, its x/z were silently dropped
  and only the roll about Y was applied).
- The fan is a full 360° in its plane and splits cleanly by the sign of the
  in-plane radial coordinate.
- ICP between the halves is a fair estimator of their relative transform (fitness
  ≥ 0.99 in all scans).

## Open questions / next steps

1. **Is the offset constant across a sweep, or does it drift?** Measure the
   half-to-half yaw in short azimuth windows and plot it around the full 360°
   (loop closure). A jump at a specific azimuth localises the seam.
2. **Does it scale with the LiDAR's own reported angle output rate?** Compare
   `SAMPLING_RATE` / firmware revisions.
3. **Practical mitigation** (if it is an instrument indexing term): apply
   ∓ 1.2° of azimuth to each half at assembly, or correct it upstream in the
   LiDAR's angular calibration.

## Tooling

| tool | purpose |
|---|---|
| `experiments/split_fans.py` | left/right half clouds + full relative transform + per-range shift |
| `experiments/split_pano.py` | left/right half **panoramas** + azimuth shift (global and per elevation band) |
| `experiments/rectify_tilt.py` | sweep the mount pitch/roll, score wall **verticality** (length-weighted Hough) |
| `experiments/beam_model_sim.py` | raycast synthetic room under cone / plane-tilt / wobble models |
| `experiments/fit_mount.py`, `fit_lidar_scale.py`, `fit_step_scale.py` | sweep the mount / range+angle scale / step scale |

> **Note:** the older `experiments/split_overlap.py` (>360° arc-overlap check) was
> **removed** — the fan-half split is the correct diagnostic for the sweep
> geometry.