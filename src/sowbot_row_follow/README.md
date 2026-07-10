# sowbot_row_follow

Monocular RGB crop-row following for **Sowbot** on the **Neo SBC**. Detects crop rows from a downward-facing USB camera, drives `/cmd_vel` during Phase 1 visual servoing, and publishes lateral offset + heading error into the Sowbot AOC navigation layer.

Derived from [visual-multi-crop-row-navigation](https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation) (Agricultural-Robotics-Bonn, BSD-2-Clause). Stripped to single RGB camera, monocular, forward-only. The TSM backend (`triangle_scan.py`) is a clean-room reimplementation of de Silva *et al.* (2024) — see [Attribution](#attribution).

---

## Package layout

```
sowbot_row_follow/
├── sowbot_row_follow/
│   ├── crop_row_node.py          # Main ROS 2 node (Neo)
│   ├── limbic_row_follow_node.py # Action server orchestrator (Limbic)
│   └── triangle_scan.py          # TSM detector backend
├── launch/
│   ├── crop_row_nav.launch.py    # Full Neo launch (camera + node + web_video_server)
│   └── row_follow.launch.py      # Minimal (no camera)
├── config/
│   └── crop_row_params.yaml      # All tunable parameters
├── test/
│   └── test_triangle_scan.py     # Unit tests (no ROS required)
└── package.xml / setup.py / setup.cfg
```

---

## Architecture: two-node, two-SBC

```
Neo SBC                                Limbic SBC
──────────────────────────────────     ─────────────────────────────────────────
usb_cam_node_exe                       limbic_row_follow_node   (action server)
  │ /devkit/camera/image_raw                │
  ▼                                         │  /row_follow/enable (SetBool)
crop_row_node                          ◄────┤
  │                                         │  /aoc/heartbeat/neo_vision
  ├─► /cmd_vel  (gated by enable)      ────►│
  ├─► /aoc/conditions/row_offset            │  /odometry/global
  ├─► /aoc/conditions/row_heading_error     │
  ├─► /aoc/heartbeat/neo_vision             │  /navigate_to_pose (Phase 2, nav2)
  └─► /caatinga_vision/row_nav/debug_image  └─► nav2 NavigateToPose action client
```

**Phase 1** — Limbic enables Neo via `/row_follow/enable`. `crop_row_node` detects rows each frame and drives `/cmd_vel` directly. Limbic monitors `/aoc/heartbeat/neo_vision` (M10) and Euclidean distance to goal.

**Phase 2** — When within `row_follow_handover_distance` of the goal, Limbic disables Neo (acknowledged), then sends the goal to nav2 `/navigate_to_pose` for precise final approach and heading alignment.

Cancel or heartbeat loss: Neo is always disabled before the action aborts.

---

## Topics

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/devkit/camera/image_raw` | `sensor_msgs/Image` | USB camera via usb_cam |
| Pub | `/aoc/conditions/row_offset` | `std_msgs/Float32` | Lateral offset [−1, 1] |
| Pub | `/aoc/conditions/row_heading_error` | `std_msgs/Float32` | Heading error (rad) |
| Pub | `/aoc/heartbeat/neo_vision` | `std_msgs/Bool` | True while rows detected (M10) |
| Pub | `/caatinga_vision/row_nav/debug_image` | `sensor_msgs/Image` | Annotated frame (Foxglove / browser) |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | Always published; gated by `/row_follow/enable` |

Services:

| Name | Type | Notes |
|------|------|-------|
| `/row_follow/enable` | `std_srvs/SetBool` | Limbic calls this to gate `/cmd_vel` output |

Action servers / clients (Limbic only):

| Name | Type | Role |
|------|------|------|
| `/limbic_row_follow` | `nav2_msgs/NavigateToPose` | **Server** — called by topo nav / devkit_ui |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | **Client** — Phase 2 final approach via nav2 |

---

## Quick start

### Build

```bash
# From workspace root (inside Docker / devkit container)
colcon build --packages-select sowbot_row_follow
source install/setup.bash
```

### Launch on Neo

```bash
# Default (scanwin detector, camera on /dev/video0)
ros2 launch sowbot_row_follow crop_row_nav.launch.py

# Specify camera device
ros2 launch sowbot_row_follow crop_row_nav.launch.py video_device:=/dev/video2

# TSM detector (two-row mode)
ros2 launch sowbot_row_follow crop_row_nav.launch.py detector:=tsm

# No camera (subscribe only — for testing with a bag)
ros2 launch sowbot_row_follow crop_row_nav.launch.py use_camera:=false
```

The launch file starts `usb_cam_node_exe`, `crop_row_node`, and `web_video_server` in a single DDS session. Browse all image streams at `http://<neo-ip>:8080`. Direct debug stream: `http://<neo-ip>:8080/stream?topic=/caatinga_vision/row_nav/debug_image`.

### Launch on Limbic

```bash
ros2 run sowbot_row_follow limbic_row_follow_node
# Or via manage.py / neo.launch.py in the devkit
```

### Enable row following manually (testing)

```bash
ros2 service call /row_follow/enable std_srvs/srv/SetBool "{data: true}"
```

---

## Camera calibration (mandatory before field use)

Edit `config/crop_row_params.yaml` and set:

| Parameter | What to measure |
|-----------|----------------|
| `camera_height_m` | Tape measure from lens to soil surface (metres) |
| `camera_tilt_deg` | Tilt below horizontal (downward = negative, e.g. `−80.0`) |

These feed directly into the Cherubini & Chaumette visual interaction matrix (`deltaz` and `tilt_angle`). Wrong values produce a systematic angular bias — the robot will still follow rows but not accurately.

---

## Algorithms

### 1. Vegetation segmentation — ExG + Otsu

Every frame is converted to an **Excess Green** index:

```
ExG = 2·G − R − B   (clamped to ≥ 0)
```

A Gaussian blur (5×5) is applied, then **Otsu's threshold** finds the optimal binary cut between soil and vegetation automatically — no hand-tuned threshold needed. The result is dilated (10×10 kernel) to merge nearby plant blobs into continuous row mass.

This step is shared by both detector backends.

### 2a. Detector: `scanwin` (default)

Classic sliding-window line fit, derived verbatim from Agricultural-Robotics-Bonn.

1. Find contour centroids of all blobs with area ≥ `min_contour_area`.
2. Divide the image width into `n_scan_windows` columns of width `window_width`.
3. In each column, collect centroids that fall inside the window.
4. Fit `x = m·y + b` (least-squares polyfit) through those centroids.
5. Keep the fit if its bottom-edge intercept lands within the image bounds.

Output: a list of `(bottom_x, slope, intercept)` row lines. The mean of all detected lines feeds the visual servo.

**When to use:** good germination density, clean images, CPU-constrained hardware. Simple to understand and tune.

### 2b. Detector: `tsm` — Triangle Scan Method

Clean-room reimplementation of de Silva *et al.* (2024), operating directly on the binary ExG mask (no contour centroids needed).

**Step 1 — Anchor scan**

Sum each column over a top-edge ROI strip of height `s·H` (default 20% of frame height, ~96 px at 480p), restricted to columns `[amin_frac·W, amax_frac·W]`. The column with the highest sum is the **apex A** — the top of the detected row. If the strip sum is below `anchor_min_sum` (end-of-row / sparse germination), the strip shifts down by `h` and retries up to `max_strip_shifts` times.

Before summing, an optional morphological opening (kernel `morph_kernel`) removes weed speckle that would otherwise bias the column sums.

Five **anchor selection modes** control which peak wins when two rows are visible in the search range:

| Mode | Behaviour |
|------|-----------|
| `argmax` | Tallest column sum (paper default, no bias) |
| `leftmost_peak` | Leftmost peak ≥ `peak_rel_thresh` × global max |
| `rightmost_peak` | Mirror |
| `spatial_prior` | Sticky: penalises distance from previous frame's anchor (`prior_lambda`) |
| `weighted` | Soft linear ramp biasing left or right side |

For a hard restriction (e.g. always scan left half only), narrow `amin_frac`/`amax_frac` directly rather than using a mode.

**Step 2 — Line scan**

Sweep candidate bottom point P along the bottom edge `[b_frac·W, c_frac·W]`, clipped to a near-vertical cone of `±max_angle_deg` around the apex. For each P, sum mask pixel values along the straight line A→P (nearest-column sampling, one sample per row). The P with the highest sum is the **base point P_r**. The detected row is the line A→P_r, returned as `x = m·y + b` (same convention as `scanwin`).

**Multi-row mode (`tsm_n_rows > 1`)**

The anchor search range `[amin_frac, amax_frac]` is partitioned into `N` equal horizontal bands. An independent TSM scan runs in each band on the same pre-processed mask (normalised and denoised once). Each band returns its own row line. The visual servo uses the mean heading of all valid bands.

When fewer than `N` bands fire (partial detection), the **Option C single-row offset** shifts `avg_row` toward the missing band by half the last observed inter-row pixel spacing (`_last_row_spacing_px / 2`). This keeps the servo aiming between the rows rather than directly at the single visible row. The debug overlay shows `1-ROW` in yellow when the offset is active.

**Temporal filter (`TSMFilter`)**

Applied per band after each raw detection. Three stages in order:

1. **Outlier gate** — if `anchor_x` or `base_x` jumps more than `jump_gate_frac·W` from the current estimate, treat the frame as a miss (likely row switch artefact or occlusion).
2. **EMA** — blend accepted frames: `x_filt = α·x_new + (1−α)·x_prev`. Lower `alpha` = smoother but slower to react.
3. **Bounded hold** — on a miss (invalid or gated), return the last estimate for up to `max_hold` consecutive frames. Beyond that, return `valid=False` so the heartbeat goes silent and Limbic aborts. **Do not raise `max_hold` to mask persistent detection loss.**

**Swap-hold debounce (single-row path only, `tsm_n_rows == 1`)**

When the raw anchor jumps by more than `tsm_swap_threshold_frac·W` in a single frame, the node suspects a row switch and starts a timer. For `tsm_swap_hold_s` seconds it keeps publishing the last accepted row geometry (amber line in debug overlay, `HOLD Xs` label). If the original row reappears the timer resets silently; if not, the new row is accepted when the timer expires. This bridges germination gaps on the preferred row without permanently locking out a genuine row switch.

At `linear_vel=0.15 m/s`, `tsm_swap_hold_s=9.0` ≈ 1.35 m of travel before switching.

**Deviations from the published paper**

`triangle_scan.py`'s Anchor-Scan/Line-Scan geometry follows de Silva *et al.* (2024) directly. Everything below it is engineering layered on top, active by default in the shipped config (`config/crop_row_params.yaml`, not `triangle_scan.py`'s own dataclass defaults, which just mirror the paper's stated parameters for reference):

| Aspect | Paper | This deployment (shipped config) | Why |
|---|---|---|---|
| Input mask | Clean segmentation mask (assumed) | ExG + Otsu vegetation index, then morphological opening (`tsm_morph_kernel: 5`) | ExG+Otsu carries shadow/weed speckle the paper's mask doesn't; opening strips small fragments before column-summing |
| Anchor selection | `argmax` — tallest column, unbiased | `spatial_prior` — sticky tracking, penalises distance from previous frame's anchor (`tsm_prior_lambda: 0.05`) | Stops the pick flipping between two near-equal peaks frame-to-frame when two rows are visible |
| Line-scan angle | Unconstrained | Clamped to a ±30° cone around the apex (`tsm_max_angle_deg: 30.0`) | Stops a sparse/noisy frame fitting a diagonal; sized to leave margin around the 8° Phase-0 entry-alignment tolerance |
| Anchor search band | Full frame width, by convention | Narrowed to 40–60% of width (`tsm_amin_frac`/`amax_frac: 0.4/0.6`) | Hard-restricts the scan to the expected central row rather than relying on anchor-selection bias alone |
| Row count | Single central row | Single row (`tsm_n_rows: 1`) — multi-row banding exists in code but isn't the shipped config | See contradiction flagged in the parameter table above |
| Line fit | Single free endpoint (line_scan) | Same (`tsm_fit_mode: "tsm"`) | A RANSAC-over-raw-pixels alternative exists in code but isn't enabled |
| Temporal smoothing | Complementary filter (not reproduced — reference doc unavailable) | Independent EMA (`α=0.4`) + outlier gate (`jump_gate_frac=0.35`) + bounded hold (`max_hold=2`) | Same role, different design |
| Row-loss handling | Not specified | Swap-hold: freezes last geometry for `tsm_swap_hold_s: 9.0`s once the anchor jumps more than `tsm_swap_threshold_frac: 0.30`×W, before accepting a new row | Bridges germination gaps without permanently locking out a genuine row switch |

Also worth noting: the reference TSM implementation (rajithadesilva/TSM) is CC BY-NC-ND, so `triangle_scan.py` is a clean-room reimplementation from the paper's written description only — even the "paper-faithful" path (`tsm_fit_mode="tsm"`, `anchor_select="argmax"`) is faithful to the algorithm as described, not verified against the original authors' code.

**When to use TSM:** sparse germination, end-of-bed conditions, noisy backgrounds. More robust than `scanwin` at the cost of slightly higher per-frame CPU (line-scan is an O(W) loop). The multi-row path (`tsm_n_rows=2`) is the recommended field configuration once the shipped-vs-recommended contradiction above is resolved.

### 3. Visual servoing controller

Derived verbatim from Cherubini & Chaumette. Inputs: mean detected `bottom_x` and `slope` of all valid rows. Outputs: angular velocity `ω`.

The **feature vector** is `[x_error, img_h/2, θ]` where:
- `x_error = bottom_x − W/2` (lateral offset from image centre)
- `θ = arctan(slope)` (heading error, rad)

The **interaction matrix** (`IntMat`) maps camera-frame feature velocities to robot velocities, parametrised by `camera_height_m` (`deltaz`) and `camera_tilt_deg` (`tilt_angle`). A 3×6 Jacobian is reduced to the 2×1 angular control law via pseudo-inverse. The result is scaled by `omega_scaler` and clipped to `±max_omega`.

The **normalised lateral offset** (`row_offset = x_error / (W/2)`, range [−1, 1]) is published separately on `/aoc/conditions/row_offset` for the AOC layer.

### 4. Heartbeat (M10)

`crop_row_node` publishes `True` on `/aoc/heartbeat/neo_vision` at 5 Hz while at least one row was detected within the last `heartbeat_timeout_s` seconds. On timeout it logs a warning and publishes `False`. `limbic_row_follow_node` aborts Phase 1 and disables Neo if the heartbeat goes dead for `heartbeat_timeout_s` (a separate, slightly longer timeout).

---

## Tuning guide

Watch `/caatinga_vision/row_nav/debug_image` in Foxglove or the browser stream while adjusting.

**Debug overlay legend:**

| Element | Meaning |
|---------|---------|
| Magenta dots | Plant centroids (contour centres) |
| Green lines | Detected row lines (individual bands or scanwin windows) |
| Amber lines + `HOLD Xs` | Swap-hold active — frozen row geometry |
| Red line | TSM multi-row averaged heading (all bands valid) |
| Yellow line + `1-ROW` | TSM averaged heading with single-row offset applied |
| Grey vertical line | Image centre reference |

**Common issues:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Lines jumping between two rows | Two rows in anchor search range, no mode bias | Use `tsm_anchor_select: leftmost_peak` or narrow `amin_frac`/`amax_frac` |
| No detection at row ends | Sparse germination below `anchor_min_sum` | Lower `tsm_anchor_min_sum` (try 4.0) |
| Ghost detections from weeds | ExG mask noisy | Increase `tsm_morph_kernel` (try 7) or `min_contour_area` |
| Robot oscillates / overcorrects | `omega_scaler` too high | Lower to 0.05; also check `camera_height_m` and `camera_tilt_deg` |
| Lines at wrong angle consistently | Camera geometry wrong | Re-measure and set `camera_height_m` and `camera_tilt_deg` |
| `1-ROW` label appears every frame | One band consistently misses | Widen `tsm_amin_frac`/`tsm_amax_frac`, check inter-row spacing vs band width |
| Swap-hold fires on legitimate row switch | `tsm_swap_hold_s` too long | Reduce to 4–6 s |

---

## Key parameters

All parameters live in `config/crop_row_params.yaml`.

### crop_row_node

| Parameter | Default | Notes |
|-----------|---------|-------|
| `camera_height_m` | `1.055` | Derived from `ifarmate.urdf.xacro` cam_mount_z, not a placeholder — **re-measure if your camera mount differs from ifarmate** |
| `camera_tilt_deg` | `-20.0` | Derived from `ifarmate.urdf.xacro` sensor pitch (0.35 rad). **The old `1.2` / `-80.0` values in this table were unconfirmed BonnBot placeholders that were never updated for Sowbot — per the config file's own comments they were very likely corrupting visual-servo steering while TSM detection itself still looked fine in `debug_image`. Do not reuse those old numbers.** |
| `image_width` / `image_height` | `640` / `480` | Must match camera output |
| `linear_vel` | `0.15` | m/s forward speed during visual servo |
| `max_omega` | `0.4` | Hard angular velocity cap (rad/s) |
| `detector` | `scanwin` | `scanwin` or `tsm`; overridden by launch arg |
| `tsm_n_rows` | `1` | Bands to scan simultaneously. **Shipped as 1 (single-row + swap-hold) even though this doc's "When to use TSM" note above calls `tsm_n_rows=2` the recommended field config — unresolved contradiction, not yet reconciled one way or the other** |
| `tsm_anchor_select` | `spatial_prior` | Sticky tracking, penalises distance from previous anchor (`tsm_prior_lambda: 0.05`) — not `leftmost_peak` |
| `tsm_swap_hold_s` | `9.0` | Seconds to hold before accepting a row switch |
| `tsm_swap_threshold_frac` | `0.30` | Anchor shift fraction that triggers hold |
| `tsm_filter_alpha` | `0.4` | EMA blend weight (lower = smoother) |
| `tsm_filter_max_hold` | `2` | Max consecutive misses before heartbeat dies |
| `tsm_max_angle_deg` | `30.0` | Near-vertical cone clamp. Sized against `row_entry_align_tolerance_deg: 8.0` in the Phase-0 alignment params below — don't tighten one without checking the other |

Values above are `config/crop_row_params.yaml` as shipped (the file `crop_row_nav.launch.py` actually loads) — not `crop_row_node.py`'s `declare_parameter()` fallbacks, which differ on several of these and only apply if the yaml key is absent.

### limbic_row_follow_node

| Parameter | Default | Notes |
|-----------|---------|-------|
| `row_follow_handover_distance` | `1.0` | Metres from goal at which Phase 2 begins |
| `heartbeat_timeout_s` | `3.0` | Abort Phase 1 if heartbeat silent this long |
| `enable_service_timeout_s` | `2.0` | Hard fault if Neo doesn't ack enable/disable |

---

## Tests

Unit tests for `triangle_scan.py` run without ROS:

```bash
python3 -m pytest src/sowbot_row_follow/test/test_triangle_scan.py -v
```

---

## Attribution

**Scan-window detector and visual servo controller** derived from:
- Ahmadi *et al.*, "Towards Autonomous Crop-Agnostic Visual Navigation in Arable Fields," arXiv:2109.11936, 2021.
- Ahmadi *et al.*, ICRA 2020.
- Original source: https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation (BSD-2-Clause)
- ROS 2 port and Sowbot adaptation: Agroecology Lab Ltd, 2026.

**Triangle Scan Method** clean-room reimplementation based solely on the published method description:
- de Silva, Cielniak, Gao, "Vision based crop row navigation under varying field conditions in arable fields," *Computers and Electronics in Agriculture*, 217, 108581, 2024. arXiv:2209.14003.
- Reference implementation (not used): https://github.com/rajithadesilva/TSM (CC BY-NC-ND 4.0). `triangle_scan.py` contains no code from that repository and carries MIT licence.
