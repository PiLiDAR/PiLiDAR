
the two most obvious issues in the project in my opinion are:
- (lib/rpicam_utils.py) how complicated white balance and exposure are measured so we can keep them stable across all photos, and how HDR brackets are shot using rpicam-still. this is all because Picamera2 was not ready back then. 
- and how preconfigured Hugin templates (e.g. hugin/template_4_AEB2-2.pto for 4 images -> 90°, HDR bracket [-4, -2, 0, +2, +4]) are used to create the panorama image. they don't account for calibration issues. the general orientation, camera intrinsics and HDR gain curve are more or less right but the exact extrinsics (angle, distortions etc) are not perfect. i was wondering if we could run feature matching and optimization in Hugin to improve the result. 

-------------------


Both of these are the primary pain points of the v1.x prototype. Below is a code-grounded analysis (verified against `lib/rpicam_utils.py`, `lib/pano_utils.py`, `lib/config.py`, `PiLiDAR.py`, and the `.pto` templates) plus the concrete fixes. A few of the "obvious" symptoms actually trace back to specific bugs, not just mechanical tolerance.

---

## Issue 1 — Camera 3A measurement & HDR bracketing (`lib/rpicam_utils.py`)

### What the code actually does today

**One subprocess per frame.** `take_photo()` builds a `rpicam-still ...` string and runs it with `subprocess.run(..., shell=True)`. Every single frame — including every AWB test frame and every bracket frame — re-initializes the sensor/CSI-2/ISP pipeline. That is the dominant source of dead time.

**The AWB "hack" (`estimate_camera_parameters()`).**
1. Takes one *auto* preview shot (`preview_dims` = 320×240) and reads `ExposureTime` + `ISOSpeedRatings` out of the EXIF via `exifread`.
2. Loops up to `max_iterations` (10) times: shoots a *manual* preview frame with fixed exposure+gain but a guessed `awbgains=[R,B]`, then compares the **mean red and mean blue** channel of that frame to the auto frame (`get_r_b_gains`) and scales `awbgains *= (R, B)` until both are within `awb_thres` (1%).

Weaknesses of this approach:
- It only closes the loop on **mean R and mean B**; green is implicitly pinned to 1. That is a 2-channel global color correction, not a real white balance, and it is sensitive to scene content (a red-dominant scene biases the result).
- It converges on a **preview** frame but the gains are then applied to full-res frames — fine in principle (gains are resolution-independent), but the reference is a tiny, denoised, auto-exposed image.
- Up to 10 extra subprocess shots, each re-initializing the pipeline.
- `set_gain` (currently `1`) triggers `exposure_time = exif_auto.exposure_time * exif_auto.gain / set_gain`, which rescales the shutter by the ISO — a subtle coupling that is easy to get wrong.

**The HDR bracket (`take_HDR_photo()`) is asymmetric on purpose.** With `AEB_stops=2`, `stop_multiplier = 2**2 = 4`:
- `step < 0` → change **shutter**: `exposure_time * 4**step` (e.g. step=-1 → ÷4 = −2 EV), gain unchanged.
- `step >= 0` → change **gain**: `gain * 4**step` (e.g. step=+1 → ×4 = +2 EV), shutter unchanged.

This is deliberate: on a rotating rig you do **not** want long shutters on the bright end (motion blur), so the bright frames are pushed with gain instead. The cost is that the bright frames carry more noise (and `--gain` in `rpicam-still` is *total* gain, so digital gain kicks in). This is the "HDR gain curve" you mentioned — it is really a **shutter-for-dark / gain-for-bright** curve, and it is not symmetric in noise.

**Bracket count depends on `AEB`, and the current config is not the 5-shot one.**
- `config.json` has `AEB=1, AEB_STOPS=2` → `range(-1, 2)` = **3 frames/angle** at [−2, 0, +2] EV → **12 images** for 4 angles. This matches `template_4_AEB1-2.pto` (3 images/angle).
- `AEB=2` → `range(-2, 3)` = **5 frames/angle** at [−4, −2, 0, +2, +4] EV → **20 images**. That is `template_4_AEB2-2.pto`.

So the "5 × 4 = 20 shots, over a minute" figure only applies to the `AEB=2` config. At the current `AEB=1` it is 12 bracket frames + up to 10 AWB preview frames.

### The fix: Picamera2 (persistent session, no EXIF, no iteration)

Picamera2 keeps one live camera session and exposes ISP metadata/controls directly, so the whole `exifread` + iterative loop disappears. Two corrections to the naive version people usually write:

1. **Actually let 3A settle** before reading metadata — `capture_metadata()` reports the *last* frame, so capture and discard a handful of frames first.
2. **Preserve the shutter-for-dark / gain-for-bright intent** (don't just scale exposure time across all stops — that reintroduces motion blur on the bright end).

```python
from picamera2 import Picamera2
import time, cv2

cam = Picamera2()
cam.configure(cam.create_still_configuration(main={"size": (4056, 3040)}))
cam.start()

# 1) Let 3A converge, then read the settled hardware values directly.
for _ in range(10):
    cam.capture_metadata(); time.sleep(0.05)
md = cam.capture_metadata()
exposure_ns   = md["ExposureTime"]   # nanoseconds (rpicam-still --shutter was microseconds!)
analogue_gain = md["AnalogueGain"]   # 1.0 = base
colour_gains  = md["ColourGains"]    # (R, B) straight from the ISP

# 2) Lock 3A for the whole 360° run — zero iteration, no EXIF parsing.
cam.set_controls({
    "AeEnable": False, "AwbEnable": False,
    "ExposureTime": exposure_ns,
    "AnalogueGain": analogue_gain,
    "ColourGains":  colour_gains,
})

# 3) HDR bracket: shutter for the dark end, analogue gain for the bright end.
MAX_AG = 16.0  # sensor-specific ceiling; keep DigitalGain at 1
def bracket(stops=(-2, 0, 2)):
    out = []
    for s in stops:
        if s < 0:
            cam.set_controls({"ExposureTime": int(exposure_ns * (2 ** s)),
                              "AnalogueGain": analogue_gain})
        else:
            cam.set_controls({"ExposureTime": exposure_ns,
                              "AnalogueGain": min(analogue_gain * (2 ** s), MAX_AG)})
        out.append(cv2.cvtColor(cam.capture_array(), cv2.COLOR_BGR2RGB))
    return out
```

Notes:
- Units differ from the old code: Picamera2 `ExposureTime` is **ns**, `rpicam-still --shutter` was **µs**.
- Use `AnalogueGain` and keep digital gain at 1 so the bright frames don't get noisier than necessary; if you need more EV than `MAX_AG` allows, either accept the cap or add a small controlled digital gain.
- Because the session stays open, the per-frame cost drops from ~1.5–3 s (pipeline re-init) to the sensor readout time, and the AWB step goes from up to 10 shots to **one** metadata read. Color temperature is now locked by the ISP across all four positions with no feedback loop.
- `exifread` can be dropped from `requirements.txt` once this lands.

---

## Issue 2 — Fixed Hugin templates vs. feature matching + optimization

### What the code actually does today

`PiLiDAR.py` calls **`hugin_stitch()` only**. That path is:
`pto_gen → pto_template → hugin_modify (resize) → hugin_executor --stitching`.
**No control points, no optimization** — it trusts the template 100%.

`hugin_refine()` (the `cpfind / cpclean / linefind / pto_var / autooptimiser` path) **exists but is dead code** — it is only referenced in the commented-out `__main__` block of `pano_utils.py`. So the "groundwork" is there but never runs in production.

### Concrete bugs in the current templates (beyond mechanical tolerance)

These are worth fixing regardless of whether we add optimization, because they explain a lot of the "not perfect" extrinsics:

1. **Capture order ≠ template order (priors land on the wrong frames).**
   - `take_HDR_photo()` emits each angle's bracket in capture order `[−4, −2, 0, +2, +4]`, and `config.imglist.extend(...)` keeps that order.
   - But `template_4_AEB2-2.pto` lists each angle as `[0, +2, −2, +4, −4]` (EV-0 first).
   - `pto_template` maps template image *i* → project image *i* **by position**. The templates contain stale Windows paths (`P:\Projekte\PiDAR\...`) that do not exist on this machine, so filename matching is impossible — the pipeline only works *because* matching is positional. That confirms the mismatch is real: the EV-0 "anchor" priors (real distortion + measured r/p/y) are being applied to the `−4` EV frame, and the actual EV-0 frame receives a placeholder.
   - It "mostly works" only because all frames at the same angle share nearly identical geometry, so the wrong-EV assignment is masked for rotation — but **distortion is applied to the wrong frames**, which is a genuine geometric error.
   - *Fix:* either sort `imglist` to match the template order, or (better) stop relying on positional templates — see the two-stage pipeline below.

2. **Lens distortion is baked into image 0 only.** In the templates, image 0 has real `Ra..Re` and `a..e` (e.g. `Ra0.4448 ... d88.3 e-13.85`), while every other image has distortion `= 0`. A real lens has the *same* distortion on every frame, so images 1–3 are left un-corrected and geometrically inconsistent with image 0. Distortion should be a **shared/linked** parameter across all images of the same lens, not a per-image value set on one frame.

3. **`hugin_refine()`'s `pto_var` string is wrong for a rotational panorama.**
   Current: `--opt TrX,TrY,TrZ,r,Eev,Ra,Rb,Rc,Rd,Re,!TrX0,!TrY0,!TrZ0,!r0,!Eev0,!Ra1,!Rb1,!Rc1,!Rd1,!Re1`
   - It **optimizes translations** `TrX,TrY,TrZ`. For a true 360° pano all images rotate about a common optical center — translations must be **locked**, or you get parallax.
   - It locks `!r0` (roll of image 0) but **not** `!p0`/`!y0`, so the anchor is not fully anchored.
   - It locks distortion on **image 1** (`!Ra1..!Re1`) — an EV+2 frame — while optimizing distortion on image 0. Inconsistent with the "image 0 is the anchor" intent.

### The fix: two-stage pipeline (fuse per angle → 4-image geometry + stitch)

Matching features across 12–20 mixed-exposure frames is fragile. The robust structure is to separate **HDR fusion** from **panorama geometry**:

```
[4 angles × N bracket frames]
        │  (1) per-angle HDR fusion (enfuse) → one well-exposed base image per angle
        ▼
[4 base images, one per 90° step]
        │  (2) pto_gen + template priors (coarse yaw/pitch/roll + shared lens distortion)
        │  (3) cpfind --multirow  →  cpclean  →  linefind
        │  (4) constrained optimization (see pto_var below)
        ▼
[nona remap + enblend] → seamless 360° equirectangular
```

Why this is better:
- **Only 4 images** go into the geometry solve, one per angle, trivially ordered by angle — this eliminates the ordering bug entirely.
- `cpfind` works best on **similarly-exposed** images; a fused (or middle-EV) base per angle gives clean, dense control points on the ~120° overlaps.
- HDR is handled by `enfuse` (per-pixel, tone-mapped) instead of Hugin's in-project stack fusion, which is simpler to reason about and to fall back on.

**Constrained optimization (`pto_var`)** for the 4-image project (indices 0–3), correcting the problems above:
```
pto_var -o proj.pto \
  --opt r,p,y,Ra,Rb,Rc,Rd,Re,a,b,c,d,e,!r0,!p0,!y0 \
  proj.pto
autooptimiser -n -o proj.pto proj.pto
```
- `r,p,y` optimized for all images, but **image 0 fully anchored** (`!r0,!p0,!y0`).
- **No `TrX/TrY/TrZ`** — translations stay locked (rotational pano, no parallax).
- Distortion (`Ra..Re,a..e`) optimized on **all** images and initialized from the *same* measured lens values, so they converge consistently instead of one frame carrying it alone. (If you prefer a single shared set, link the distortion across images in the project before optimizing.)
- Add `Eev` to the list only if you keep HDR inside Hugin rather than pre-fusing.

**Graceful fallback.** If the scene is textureless (blank walls) and `cpfind` yields too few control points (e.g. < ~5 per overlapping pair), skip optimization and stitch with the template priors as-is — log a warning rather than crashing. This is the same "fallback grace" idea, but keyed on an actual control-point count from `cpfind`/`cpclean` output.

**Optional: match on the middle-EV frame instead of a fused image.** The EV-0 frame of each bracket is already a single well-exposed image and is what the templates designate as the geometric anchor. Matching on the four EV-0 frames is the simplest variant; pre-fusion is the upgrade for high-contrast or texture-poor scenes where the wider dynamic range helps `cpfind`.

---

## Roadmap integration

- **Phase 1 (Hardware Modernization):** replace `lib/rpicam_utils.py` with a Picamera2-based capture module — persistent session, 3A settled then locked from metadata, shutter/gain bracketing. Drops `exifread` and the iterative AWB loop.
- **Phase 2/3 (Calibration & Pipeline Engine):**
  - Fix the template bugs first (image ordering, shared distortion) even before adding optimization — they are cheap and remove real artifacts.
  - Wire the currently-dead `hugin_refine()` into the pipeline as the two-stage fuse→optimize→stitch flow, with the corrected `pto_var` and a control-point-count fallback.
  - Persist the optimized per-lens distortion + per-mount extrinsics into the scan `metadata.json` sidecar (Phase 2 data standards) so future scans can seed from a measured prior instead of a hand-authored template.
