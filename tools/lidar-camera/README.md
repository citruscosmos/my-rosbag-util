# lidar-camera

Tools for extracting camera images and LiDAR point clouds from MCAP bags, projecting LiDAR onto camera images, and manually fine-tuning extrinsic parameters.

## Dependencies

- `mcap` — MCAP file reading
- `rclpy` / `sensor_msgs` — ROS 2 message deserialization (requires a ROS 2 environment)
- `numpy`
- `opencv-python`
- `pyyaml`
- `PySide6`, `matplotlib` — required by `tune_extrinsic.py` only

## Scripts

| Script | Role |
|--------|------|
| `run_workflow.py` | **Run the full pipeline in one command** |
| `extract_cameras.py` | Extract camera images (JPEG) from MCAP |
| `extract_lidar_pcd.py` | Extract LiDAR point clouds (PCD) from MCAP |
| `project_lidar_to_cam.py` | Project LiDAR point clouds onto camera images |
| `tune_extrinsic.py` | GUI tool to inspect and manually adjust extrinsic parameters |

---

## Quick Start: Run the full pipeline

```bash
python3 run_workflow.py <mcap> [--start <unix_sec>] [--end <unix_sec>] [--distortion-model <model>] [--alpha <float>]
```

**Arguments:**

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `mcap` | ✓ | — | Input MCAP file path |
| `--start` | | `None` | Start time in Unix seconds |
| `--end` | | `None` | End time in Unix seconds |
| `--distortion-model` | | auto | `rational_polynomial` or `equidistant`; omit to use each camera's `camera_info.json` |
| `--alpha` | | `0.45` | Point opacity for projection (0–1) |

**Output** is written to `<mcap_dir>/<mcap_stem>_proj/`:

```
recording_proj/
  camera0/                  ← extracted JPEG frames + camera_info.json
  camera1/ ... camera11/
  lidar_front/              ← extracted PCD frames
  lidar_left/
  lidar_rear/
  lidar_right/
  proj_camera0/             ← LiDAR-on-camera projection
    distort/
    undistort/
  proj_camera1/ ...
```

Cameras without extracted images are skipped automatically.

**Example:**

```bash
python3 tools/lidar-camera/run_workflow.py \
  /path/to/recording.mcap \
  --start 1782377330 \
  --end   1782377340
```

---

## Command Flow

### Step 1: Extract camera images

Extracts compressed images (JPEG) from an MCAP file for the specified cameras.
File names are nanosecond timestamps from `header.stamp` (`{sec}{nanosec:09d}.jpg`).
Each camera directory also receives a `camera_info.json` containing intrinsics (`K`, `D`, `P`, `width`, `height`, `distortion_model`).

```bash
python3 extract_cameras.py <mcap> <out_root> [--cams <camera IDs>] [--limit <N>]
```

**Arguments:**

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `mcap` | ✓ | — | Input MCAP file path |
| `out_root` | ✓ | — | Output root directory |
| `--cams` | | auto-detect | Comma-separated camera IDs to extract; omit to use all cameras found in the MCAP |
| `--limit` | | `0` (unlimited) | Maximum frames per camera |
| `--start` | | `None` | Start time in Unix seconds |
| `--end` | | `None` | End time in Unix seconds |

**Topics:** `/sensing/camera/camera{N}/image_raw/compressed`

**Examples:**

```bash
# Extract all cameras (auto-detect)
python3 extract_cameras.py sample.mcap ./output

# Extract 10 frames each from camera2 and camera3 (for validation)
python3 extract_cameras.py sample.mcap ./output --cams 2,3 --limit 10
```

**Output structure:**

```
output/
  camera0/
    camera_info.json
    1751234567000000000.jpg
    ...
  camera1/ ...
```

---

### Step 2: Extract LiDAR point clouds

Extracts point clouds from four LiDAR sensors in binary PCD format (PCD v0.7, FIELDS x y z intensity, float32). Non-finite points (NaN/Inf) are removed.

```bash
python3 extract_lidar_pcd.py <mcap> <out_root> [--limit <N>]
```

**Arguments:**

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `mcap` | ✓ | — | Input MCAP file path |
| `out_root` | ✓ | — | Output root directory |
| `--limit` | | `0` (unlimited) | Maximum frames per LiDAR |
| `--start` | | `None` | Start time in Unix seconds |
| `--end` | | `None` | End time in Unix seconds |

**Topics:**

| Topic | Output directory |
|-------|-----------------|
| `/sensing/lidar/front/seyond_points` | `lidar_front/` |
| `/sensing/lidar/left/seyond_points` | `lidar_left/` |
| `/sensing/lidar/rear/seyond_points` | `lidar_rear/` |
| `/sensing/lidar/right/seyond_points` | `lidar_right/` |

**Examples:**

```bash
# Extract all frames
python3 extract_lidar_pcd.py sample.mcap ./output

# Extract 5 frames per LiDAR (for validation)
python3 extract_lidar_pcd.py sample.mcap ./output --limit 5
```

---

### Step 3: Project LiDAR onto camera images

Uses the images and point clouds from Steps 1 and 2 to overlay LiDAR points on camera images.
For each LiDAR frame, the nearest-timestamp camera image is matched automatically.
Per-camera intrinsics are loaded from `camera_info.json` when available.

```bash
python3 project_lidar_to_cam.py <cam> <out_root> [options]
```

**Arguments:**

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `cam` | ✓ | — | Target camera (`camera0` – `camera11`) |
| `out_root` | ✓ | — | Output root directory |
| `--sample` | | `0` (disabled) | Process N evenly-spaced frames (for validation) |
| `--limit` | | `0` (unlimited) | Stop after N frames from the beginning |
| `--alpha` | | `0.45` | Point opacity (0 = fully transparent, 1 = opaque) |
| `--start` | | `None` | Start time in Unix seconds |
| `--end` | | `None` | End time in Unix seconds |
| `--cam-dir` | | `CAM_CONFIGS` | Camera image directory (overrides hardcoded path) |
| `--lidar-dir` | | `CAM_CONFIGS` | LiDAR PCD directory (overrides hardcoded path) |
| `--distortion-model` | | auto | `rational_polynomial` or `equidistant` (fisheye); auto-detected from `camera_info.json` |
| `--tf-yaml` | | — | Load extrinsic parameters from `multi_tf_static.yaml` instead of hardcoded `CAM_CONFIGS` |
| `--base-dir` | | `CAM_CONFIGS` path | Base directory for camera/LiDAR data when using `--tf-yaml` |

**Camera–LiDAR pairs (hardcoded in `CAM_CONFIGS`):**

| LiDAR | Cameras |
|-------|---------|
| `lidar_front` | camera0, camera1, camera8 |
| `lidar_right` | camera2, camera3, camera9 |
| `lidar_rear` | camera4, camera5, camera10 |
| `lidar_left` | camera6, camera7, camera11 |

When `--tf-yaml` is given, camera–LiDAR pairs and extrinsics are read from the YAML instead.

**Output modes:**

| Directory | Processing |
|-----------|-----------|
| `distort/` | Original image + points projected with the distortion model |
| `undistort/` | Undistorted image (`cv2.remap`) + pinhole projection with `new_K` |

Point colors are mapped via JET colormap with intensity normalized to 0–40.

**Examples:**

```bash
# All frames for camera2 (uses hardcoded CAM_CONFIGS paths)
python3 project_lidar_to_cam.py camera2 ./proj_output

# 10 evenly-spaced frames (validation)
python3 project_lidar_to_cam.py camera2 ./proj_output --sample 10

# Load extrinsics from YAML, override data base directory
python3 project_lidar_to_cam.py camera0 ./proj_output \
  --tf-yaml /path/to/multi_tf_static.yaml \
  --base-dir /data/dump
```

---

### Step 4 (optional): Manually adjust extrinsic parameters

GUI tool for visually inspecting the LiDAR-camera alignment and fine-tuning the extrinsic transform.
Input images should be rectified (e.g. from `undistort/`).
Output is saved in `multi_tf_static.yaml` format and can be fed directly back to `project_lidar_to_cam.py` via `--tf-yaml`.

```bash
python3 tune_extrinsic.py --image <jpg> --points <pcd> --cam-dir <dir> [options]
```

**Arguments:**

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--image` | ✓ | — | Rectified image file (e.g. from `proj_cameraN/undistort/`) |
| `--points` | ✓ | — | LiDAR point cloud (`.pcd` from `extract_lidar_pcd.py`, or `.npy` Nx3) |
| `--cam-dir` | ✓* | — | Directory with `camera_info.json`; derives `new_K` via `build_undistort` |
| `--K` | ✓* | — | `new_K` 3×3 as `.npy` or flat-list YAML; used when `--cam-dir` is omitted |
| `--distortion-model` | | auto | Override distortion model when using `--cam-dir` |
| `--tf-yaml` | | — | `multi_tf_static.yaml` to load the initial extrinsic transform |
| `--cam` | | — | Camera name (e.g. `camera0`); required with `--tf-yaml` |
| `--base-dir` | | — | Base data directory used by `_load_tf_yaml` |
| `--init-T` | | identity | Initial `T_col_lr` 4×4 `.npy`; lower priority than `--tf-yaml` |
| `--lidar-frame` | | auto | Lidar frame name in the output YAML (auto-derived from `--tf-yaml`) |
| `--cam-frame` | | `--cam` | Camera frame name in the output YAML |
| `--out` | | `extrinsic_adjust.yaml` | Output YAML path |

*One of `--cam-dir` or `--K` is required.

**GUI features:**

- **Point overlay** — LiDAR points are depth-colored (JET, `jet_r`) and overlaid on the rectified image.
- **Sliders** — Six-DOF fine adjustment (±10 cm / ±5°) with live re-projection on every change. Arrow keys move the selected slider by one step (0.5 mm / 0.02°). Click "基準を現在値にリセット" to re-zero the sliders around the current pose, extending the effective range.
- **Pair mode** — Click a projected point (selects the nearest 3D point) then click the correct image location to register a 2D–3D correspondence. Four or more pairs enable "solvePnP で粗合わせ" for a coarse alignment.
- **Reprojection error** — Mean reprojection error over registered pairs is shown live.
- **YAML save** — Saves in `multi_tf_static.yaml` format and prints the equivalent `ros2 run tf2_ros static_transform_publisher` command.

**Typical workflow:**

```bash
# 1. Run the main pipeline to produce undistorted images and PCD files
python3 run_workflow.py recording.mcap --start 1782377330 --end 1782377340

# 2. Open the GUI tuner with initial extrinsics from the YAML
python3 tune_extrinsic.py \
  --image   recording_proj/proj_camera0/undistort/1782377335000000000.jpg \
  --points  recording_proj/lidar_front/1782377335000000000.pcd \
  --cam-dir recording_proj/camera0 \
  --tf-yaml /path/to/multi_tf_static.yaml --cam camera0 \
  --out     camera0_adjusted.yaml

# 3. Re-project using the adjusted extrinsics
python3 project_lidar_to_cam.py camera0 ./proj_adjusted \
  --cam-dir   recording_proj/camera0 \
  --lidar-dir recording_proj/lidar_front \
  --tf-yaml   camera0_adjusted.yaml
```

---

## Overall Flow

```
sample.mcap
    │
    ├─ extract_cameras.py   ──→  output/camera{N}/*.jpg + camera_info.json
    │
    └─ extract_lidar_pcd.py ──→  output/lidar_{front,left,rear,right}/*.pcd
                                           │
                                project_lidar_to_cam.py  [--tf-yaml for extrinsics]
                                           │
                              proj_output/{distort,undistort}/*.jpg
                                           │
                               (if alignment needs adjustment)
                                           │
                                 tune_extrinsic.py  ──→  *_adjusted.yaml
                                           │
                                project_lidar_to_cam.py --tf-yaml *_adjusted.yaml
```

---

## Known Limitations

- When using `--distortion-model equidistant`, `D` must have 4 coefficients (k1, k2, k3, k4); values from `camera_info.json` are used automatically when `--cam-dir` is specified.
- The extrinsic tuner (`tune_extrinsic.py`) adjusts one camera at a time. The slider range is ±10 cm / ±5°; click "基準を現在値にリセット" after a coarse `solvePnP` alignment to extend the effective range for fine-tuning.
