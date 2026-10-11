# IMU gravity leveling — problem statement & current state

> **Status:** *experimental / not reliable yet.* `IMU.LEVEL` is on by default but
> the leveling only removes ~60 % of the tilt (measured against the floor). This
> document is a hand-off so the problem can be attacked independently.

## Goal

Use the MPU6050 to measure the scanner's tilt relative to gravity and **rotate
the assembled point cloud so it is level** — i.e. so that floors/ceilings are
horizontal in the exported `.ply`, regardless of how the tripod is tilted.

Target: a tilted scan (~10–15°) should, after leveling, have a floor plane within
**~1°** of horizontal.

## Coordinate frames

There are four frames in play. Getting these right is the whole problem.

### 1. Scanner / device frame (what the cloud is assembled in)

- **Z = up** (the turntable's rotation axis).
- The LiDAR is mounted **vertically** on the back of the scanner, so a 2D LiDAR
  plane's normal ("local up") is perpendicular to Z; the plane spans X–Y.
- `merge_2D_points()` builds the cloud by inserting `Y=0` into each 2D fan and
  revolving it about `up_vector=(0,0,1)` by the negative stepper angle.
- So **by construction the cloud's up is +Z**, and a *level* rig produces a
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

All saved in `scans/<id>/<id>_lidar.pkl`. (`quaternions` is only present when the
IMU initialised — it failed silently in older scans.)

Crucially: the **stepper angle is trusted**; the **IMU yaw is not** (the DMP
under-reports rotation — a commanded 5°/step reads back ~2.3°; see "Known IMU
limitations").

## What we currently compute

### Up vector (mean gravity in the IMU body frame)

`imu_up_vector(quaternions)`:

```
g_body[k] = R(q_k)^T @ [0,0,1]          # gravity in the IMU body frame
up_body   = normalize(mean_k g_body)    # unit vector
```

For a **level** rig, `g_body` is constant and equals the turntable axis in the
IMU frame. For a **tilted** rig, gravity sweeps a cone about the turntable axis
as the platform rotates, so this **mean is biased** (the sweep is only 180°, not
a full circle). The true axis is better estimated by `fit_turntable_axis()`
(the common axis of the incremental rotations, or the gravity-cone plane normal).

### Mounting rotation (IMU → cloud)

`IMU.MOUNTING` in `config.json` is a **3×3 rotation** that maps a vector from the
IMU frame into the cloud frame. Currently it is a *data-derived* matrix,
calibrated once (2026-10-11) by solving, over two scans (one level, one tilted
~11°), the correspondences:

```
turntable axis (in IMU body)  ->  [0,0,1]     (cloud up)
gravity g_body (in IMU body)  ->  floor normal (measured from the cloud)
```

using `scipy.spatial.transform.Rotation.align_vectors`. Current value:

```
[[-0.1042, -0.0445,  0.9936],
 [ 0.9945,  0.0000,  0.1043],
 [-0.0046,  0.9990,  0.0443]]
```

(The idealised, hand-guessed permutation `X→Y, Y→Z, Z→X` was ~90° wrong in
azimuth and did **not** work — that is why it is now solved from data.)

### Up vector in the cloud frame

`cloud_up_vector(quaternions, mounting)` = `mounting @ imu_up_vector(quaternions)`.

### Applying the correction to the cloud

Applied **once, at the very end** of `postprocess_3D()` — after vertex colouring
(the panorama lookup is referenced to the device frame, so colouring must happen
first), just before saving:

```python
leveling = np.eye(4)
leveling[:3, :3] = leveling_rotation(imu_up).as_matrix()   # maps imu_up -> [0,0,1]
pcd = transform(pcd, transformation=leveling)
```

`leveling_rotation(up)` returns the rotation taking `up` onto `[0,0,1]`
(`Rotation.align_vectors([Z],[up])`, minimal rotation). Assembling in device
space and rotating once at the end is equivalent to rotating every plane, and
keeps the pano colouring valid.

## How success is measured

The honest test is **"is the floor horizontal?"**, not "does `up` map to Z"
(which is trivially true by construction and proves nothing).

Procedure used:

1. Assemble the cloud (device space).
2. Estimate the **floor plane normal**: take the lowest ~15 % of points in Z and
   fit a plane by **PCA** (smallest singular vector). ⚠️ This estimate is crude
   and sensitive to the slab choice — a better method is wanted.
3. Report the angle between that normal and `[0,0,1]` for:
   - the raw cloud,
   - the cloud after `leveling_rotation(cloud_up_vector(...))`.

## Known IMU limitations

- **Yaw under-reporting:** the 6-axis DMP fusion has no absolute heading, and in
  practice it also under-reports *relative* rotation (5°/step → ~2.3°). The axis
  *direction* is good; the *magnitude* is not. Use the stepper angle for the
  sweep, the IMU only for the axis/tilt.
- **Biased mean gravity:** with a 180° sweep the mean of `g_body` is not exactly
  the axis (see above); prefer `fit_turntable_axis().c_body`.
- **Old firmware failure mode:** the DMP FIFO could wedge (`OSError`) → no
  quaternions. Fixed by moving to a hardware I²C bus (`i2c6`, GPIO22/23) + block
  reads; a fresh scan now yields valid unit quaternions.

## Current measurements

| scan | raw floor tilt | after leveling | notes |
|---|---|---|---|
| `261011-1953` | 2.6° | 0.6–1.8° | rig roughly level |
| `261011-2031` | 10.3° | **4.2° (exported PLY)** | rig tilted ~11° down |

So leveling currently removes only ~60 % of the tilt in the real export — it is
a **partial** correction, not a fix.

## Where it likely goes wrong (candidate hypotheses)

1. **The mounting is under-constrained / mis-calibrated.** It was solved from only
   two scans using a *crude PCA floor estimate*. A robust calibration (a large
   flat reference plane, or several tilts, least-squares over many
   correspondences) is needed.
2. **The up vector should be the turntable axis, not the mean gravity.** Using
   `fit_turntable_axis().c_body` (sign-aligned to gravity) instead of
   `imu_up_vector()` may be more correct for a tilted rig.
3. **The cloud frame's up may not be exactly the revolve axis** because of the
   LiDAR's mechanical offset (`LIDAR.Y_OFFSET` etc.) — worth checking whether the
   assembly introduces a small tilt of its own.
4. **The floor-normal estimator is unreliable** and may be partly guessing the
   metric rather than the method being wrong.

## Open question (raised by the user)

Should we compute, instead of a single **up vector**, a **full quaternion for the
scan at stepper angle = 0°** (the scanner's orientation in space at the sweep
start), and apply *that* to the cloud? This would carry the full orientation
(including the mounting azimuth) in one object and could subsume the separate
mounting matrix. It is untested — a good starting point for the next attempt.

## Code map

| piece | location |
|---|---|
| DMP read + block FIFO | `lib/imu_driver.py` |
| axis fit, up vector, mounting, leveling | `lib/imu_orientation.py` |
| applied in the pipeline | `lib/pointcloud.py` (`process_raw` / `postprocess_3D`) |
| config | `config.json` → `IMU` (`i2c_bus`, `MOUNTING`, `LEVEL`) |
| tests | `tests/test_imu_axis.py` |
| probe / data capture | `experiments/up_vector_probe.py` |
| raw quaternion logging | `experiments/imu_orientation/store_quaternions.py` |

## Data on hand for the next session

- `scans/261011-1953` — rig roughly level (tilt ~2.8°), 984 quaternions.
- `scans/261011-2031` — rig tilted down ~11°, 984 quaternions.
- Both downloaded locally under `scans/`.