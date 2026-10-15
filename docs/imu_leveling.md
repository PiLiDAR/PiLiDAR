# IMU gravity leveling — how it works

> **Status:** *working.* `IMU.LEVEL` (default on) levels the cloud to gravity
> after vertex colouring. On a real scan tilted 18.4° the floor comes out **0.10°**
> off horizontal; a level scan keeps its ~2° noise floor. The residual is
> dominated by the cross-scan consistency of the mounting (~2°, see "Remaining
> accuracy limit").

## Goal

Use the MPU6050 to measure the scanner's tilt relative to gravity and **rotate
the assembled point cloud so it is level** — i.e. so that floors/ceilings are
horizontal in the exported `.ply`, regardless of how the tripod is tilted.

Target: a tilted scan (~10–15°) should, after leveling, have a floor plane within
**~1°** of horizontal.

## Coordinate frames

There are four frames in play. Getting these right is the whole problem.

### 1. Scanner / object frame (what the cloud is assembled in)

**Object space** — independent of gravity. The cloud and the LiDAR/colour
panorama are built here; gravity leveling is a separate, later step.

| axis | meaning |
|---|---|
| **+Z** | scanner **up** = turntable rotation axis (floor at −Z, ceiling at +Z) |
| **+Y** | the **forward** direction (the direction the camera looks at stepper 0°) |
| **X** | the remaining horizontal axis (left/right) |

- The camera nodal point sits **on the rotation axis** at the origin and looks
  **+Y** (its pan is 0° by definition). The LiDAR is mounted **back-to-back** with
  it, so it faces **−Y** — the same axis, opposite sign — offset from the axis.
- At stepper 0° the LiDAR's vertical disc therefore lies in the **X–Z plane**, and
  its **roll axis is Y** (the disc normal): spin about Y tilts the fan within that
  plane. As the turntable turns, the disc (and its normal) revolve about +Z.
- `merge_2D_points()` builds the cloud by inserting `Y=0` into each 2D fan and
  revolving it about `up_vector=(0,0,1)` by the negative stepper angle.
- The LiDAR mount (`3D.CALIBRATION` in config) is one rigid transform in scanner
  space:
  - `POSITION` = `[x, y, z]` mm — the LiDAR origin (X = in-disc/left-right, **Y =
    forward/mount axis** (the LiDAR's −Y side, so -37.5), **Z = height above the
    camera** (+41.9)).
  - `ROTATION` = `[rx, ry, rz]` degrees — the mount rotation; the **roll about Y**
    (`ry = -1.05`) spins the upright disc in its own plane.
  - X and the other rotations are assumed 0.
- So **by construction the cloud's up is +Z**; a *level* rig then produces a
  cloud whose floor is horizontal.

### 2. IMU frame (the MPU6050 body)

Physical mounting, seen from the scanner's perspective:

| IMU axis | points |
|---|---|
| **X** | left |
| **Y** | up |
| **Z** | forward |

The IMU sits **on the rotating platform**, so it revolves with the turntable.

### 3. DMP world frame (the MPU6050's internal reference)

The DMP outputs a quaternion `q` per sample (wxyz, int16, unit norm ≈ 16384) that
rotates **IMU body → DMP world**. The DMP world **Z is gravity up** (it is
accel/gyro-fused, no magnetometer, so yaw drifts but the Z reference is gravity).

### 4. Cloud frame == scanner frame (see #1)

## Data recorded during a scan

`PiLiDAR.py` records, for **each LiDAR plane** (~1000 per 180° sweep):

- `z_angles` — the **stepper angle** at that plane (the true, clean rotation),
- `cartesian` — the 2D fan (X, I columns) for that plane,
- `quaternions` — one MPU6050 DMP quaternion `(w,x,y,z)` int16 per plane.

All saved in `scans/<id>/<id>_lidar.npz` (+ `.json` metadata; legacy `.pkl`
supported). `quaternions` is present only when the IMU initialised; the flaky
DMP upload (see below) can silently drop it, so check for it before expecting a
leveled export.

Crucially: the **stepper angle is trusted**; the **IMU yaw is not** (the DMP
under-reports rotation — a commanded 5°/step reads back ~2.3°; see "Known facts
& limitations").

Note: the first plane of a sweep is a partial **warm-up** frame and is dropped
(see "Known facts"); that leaves **one more quaternion than z_angles**, with the
extra leading sample at a stale pre-sweep attitude.

## What we compute

### Gravity at the reference attitude (the leveling "up")

`reference_gravity(quaternions, z_angles)`:

```
g_body[k] = R(q_k)^T @ [0,0,1]          # gravity in the IMU body frame
up_body   = normalize(mean of g_body over the samples with |z_angle - 0°| <= 2°)
```

The IMU revolves about the turntable axis, so on a **tilted** rig gravity traces
a **cone** about that axis in the body frame. Over the 180° sweep only a
**half**-cone is covered, so the whole-sweep *mean* — `imu_up_vector()`, the old
method — lies **inside** the cone and **under-reports the tilt** (it removed only
~60 %). Sampling gravity at a single attitude (stepper angle ≈ 0°, averaged over
a ±2° window) is not cone-biased.

`z_angles` is aligned first: if there is one more quaternion than plane (the
dropped warm-up plane) the extra **leading** sample is discarded before indexing.

`imu_up_vector()` is kept but is **deprecated for leveling** — it is the biased
mean.

### Mounting rotation (IMU → cloud)

`IMU.MOUNTING` in `config.json` maps a vector from the IMU frame into the cloud
frame. It is calibration data, solved (2026-10-11) from a **tilted** scan via
`experiments/calibrate_mounting.py` (→ `solve_mounting()`), from two
correspondences:

```
turntable axis  (body, fit_turntable_axis().c_body_accel)  ->  [0,0,1]       (cloud up)
reference gravity (body, reference_gravity)                ->  floor normal  (measured)
```

with `scipy.spatial.transform.Rotation.align_vectors`. It is stored in
`config.json` as **euler angles (xyz, degrees)** — human-readable — and converted
to a 3x3 matrix by `mounting_matrix()`. Current value:

```
[86.486315, 0.444054, -175.170117]
```

Two things matter here:

- The axis is mapped to **+Z**, but aligning gravity to +Z is what makes the
  floor horizontal. The cloud up-axis is already +Z *by construction* (the fans
  are revolved about `(0,0,1)`), so leveling by the axis alone would be a no-op.
- The old idealised permutation `X→Y, Y→Z, Z→X` was ~90° wrong in azimuth and
  did not work; and the previous matrix was fit to the *biased mean* gravity
  (~89° off in azimuth from the corrected gravity) — it "worked" only because
  the bias cancelled that error. Both are gone.

### Applying the correction

Once, at the end of `postprocess_3D()` — after vertex colouring (the pano lookup
is referenced to the device frame, so colouring must happen first), just before
saving:

```python
cloud_up = MOUNTING @ reference_gravity(quaternions, z_angles)
leveling = np.eye(4)
leveling[:3, :3] = leveling_rotation(cloud_up).as_matrix()   # maps cloud_up -> [0,0,1]
pcd = transform(pcd, transformation=leveling)
```

`leveling_rotation(up)` is `Rotation.align_vectors([Z],[up])` (minimal rotation).
Assembling in device space and rotating once at the end equals rotating every
plane, and keeps the pano colouring valid.

## How success is measured

The honest test is **"is the floor horizontal?"**, not "does `up` map to Z"
(trivially true by construction, proves nothing).

`experiments/floor_tilt.py` assembles the cloud and estimates the floor normal
with a **RANSAC plane fit** (Open3D `segment_plane`) over the lowest ~15 % of
points by Z — robust, unlike the earlier PCA/smallest-singular-vector slab
estimate. It reports the angle between that normal and +Z for the raw and the
leveled cloud. `experiments/calibrate_mounting.py` uses the same estimator.

## Known facts & limitations

- **Yaw under-reporting:** the 6-axis DMP has no absolute heading and also
  under-reports *relative* rotation (commanded 5°/step reads back ~2.3°). The
  axis *direction* is good, the magnitude is not — use the stepper angle for the
  sweep, the IMU only for the axis/tilt.
- **Flaky DMP init (I²C EIO):** the DMP firmware upload intermittently fails with
  `OSError` `EIO`/`EREMOTEIO`, even though the chip answers at `0x68`
  (single-byte register reads/writes always work — only the block upload fails).
  It usually succeeds within ~3–10 attempts; `MPU6050Wrapper` retries up to 10×.
  A failed init silently drops the IMU for the whole scan (no `quaternions` → no
  leveling), so check the pickle. A device reset does not reliably help; suspect
  a hardware cause (jumper/pull-ups on GPIO22/23).
- **Warm-up first plane:** the stream is live and the turntable already turning at
  scan start, so the first plane is a partial arc labelled with a **stale**
  pre-sweep `z_angle`. `Lidar._finalize_plane()` drops it (anchor mode). It also
  leaves one extra (stale) leading quaternion, which `reference_gravity()`
  discards.
- **Turntable axis:** the reliable body-frame estimate is
  `fit_turntable_axis().c_body_accel` (gravity-cone fit). The incremental
  `c_body` has ~14° spread and is less consistent between scans.

## Measurements

Floor tilt = RANSAC floor-normal vs +Z (`experiments/floor_tilt.py`).

| scan | rig tilt (raw floor) | after leveling (new) | notes |
|---|---|---|---|
| `261011-2329` | 18.36° | **0.10°** | used to (re)solve `IMU.MOUNTING` |
| `261011-2354` | 2.18° | 1.60° | level rig; residual ≈ the ~2° ceiling |
| `261011-2345` | 2.19° | (no quats) | IMU init failed → not leveled |

For comparison, on `261011-2329` the old mean-gravity method left **21°**.

## Remaining accuracy limit (~2°)

The mounting solved on one scan does not fully generalise: solved on the 18.4°
scan (`2329`) it reaches 0.10° there, but on the level scan (`2354`) it leaves
~1.6°. The cause is the body-frame turntable axis — `c_body_accel` differs ~2°
between the two scans (its `spread_deg` is ~14° in *both*), so the two
correspondences that define the mounting are only good to ~2°.

A level scan cannot discriminate the methods (old and new both land near the ~2°
noise floor); the tilted scan is where the correction matters. Accepted for now.

Possible improvements (only if a tighter target is needed): a more robust axis
estimate (longer sweep / accel-only), or a joint solve over several scans.

## Open question (resolved)

The old "apply a full quaternion at stepper angle 0° instead of an up-vector" idea
is unnecessary: only the **direction of gravity** matters for "floor
horizontal", and the mounting already carries the frame azimuth. Yaw is not used
(no magnetometer), so a single gravity vector suffices.

## Code map

| piece | location |
|---|---|
| DMP read + block FIFO (+ init retries) | `lib/imu_driver.py` |
| axis fit, reference gravity, mounting, leveling | `lib/imu_orientation.py` |
| applied in the pipeline | `lib/pointcloud.py` (`process_raw` / `postprocess_3D`) |
| warm-up first-plane drop | `lib/lidar_driver.py` (`_finalize_plane`) |
| config | `config.json` → `IMU` (`i2c_bus`, `MOUNTING`, `LEVEL`) |
| tests (synthetic geometry) | `tests/test_imu_axis.py` |
| mount calibration | `experiments/calibrate_mounting.py` |
| floor-tilt metric | `experiments/floor_tilt.py` |

## Scans on hand

- `261011-2329` — tilted ~18°, 984 quaternions (well-conditioned; used to solve
  `IMU.MOUNTING`).
- `261011-2354` — level, 986 quaternions (validates the mounting).
- `261011-2345` — level, **no** quaternions (IMU init failed).
- `240827-1322` — old `lidar/`-slice-directory scan: no pickle, no quaternions.