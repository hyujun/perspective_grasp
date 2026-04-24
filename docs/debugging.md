# Pipeline Debugging

Symptom-driven playbook for diagnosing the perception pipeline end-to-end:
Phase 1–3 host nodes, Phase 4 Docker ML nodes, fusion, mode control,
per-camera streams, and the debug visualizer that ties them together.

> This file used to live at
> `packages/infrastructure/perception_debug_visualizer/docs/DEBUGGING.md`.
> It was moved here because the playbook is workspace-wide — the debug
> visualizer is just the primary tool you use to read the pipeline state.

See also:
- [running.md](running.md) — how to launch the pipeline and camera drivers
- [architecture.md](architecture.md) — topic / TF / QoS reference
- [perception_debug_visualizer README](../packages/infrastructure/perception_debug_visualizer/README.md) — node internals (parameters, subscribe topics, CMake targets)
- `packages/bringup/perception_bringup/config/camera_config*.yaml` — source of truth for camera namespaces

---

## 1. What the debug visualizer does

Single fan-in debug visualizer. Subscribes to **all** configured cameras and
republishes a BGR8 annotated image (bbox + mode + camera tag) for **one**
selected camera on `/debug/image`.

- Per-camera subscriptions are built from `camera_namespaces`, composed with
  `image_topic_suffix` / `detection_topic_suffix`.
- Only the camera at `active_camera_index` gets overlayed + republished.
- `active_camera_index` is hot-swappable: change it at runtime without
  relaunching.

### Topic map

| Kind       | Topic template                                   | QoS                    | Source          |
|------------|--------------------------------------------------|------------------------|-----------------|
| Per-cam    | `/{ns}/{image_topic_suffix}` (default `camera/color/image_raw`) | sensor / BEST_EFFORT   | RealSense driver |
| Per-cam    | `/{ns}/{detection_topic_suffix}` (default `yolo/detections`)    | sensor / BEST_EFFORT   | `yolo_byte_tracker` |
| Global     | `/smoother/smoothed_poses`                       | sensor / BEST_EFFORT   | `pose_graph_smoother` |
| Global     | `/meta_controller/active_pipeline`               | reliable / depth 1     | `perception_meta_controller` |
| Output     | `/debug/image`                                   | sensor / BEST_EFFORT   | **this node**   |

`{ns}` is the namespace from the camera config. For a 1-cam config with
`namespace: ""`, the per-camera topics collapse to `/camera/color/image_raw`
and `/yolo/detections` (no leading namespace segment).

---

## 2. Launch recipes

All commands assume the workspace is sourced (see [running.md](running.md#prerequisites)).

### 2.1 Single camera, node only

```bash
ros2 launch perception_debug_visualizer debug_visualizer.launch.py
```

Uses defaults from `visualizer_params.yaml` → one camera at the root
namespace. Publishes `/debug/image`, no GUI.

### 2.2 Single camera + rqt_image_view window

```bash
ros2 launch perception_debug_visualizer debug_visualizer.launch.py gui:=true
```

### 2.3 Single camera + RViz preset (TF + image + merged cloud)

```bash
ros2 launch perception_debug_visualizer debug_visualizer.launch.py rviz:=true
```

### 2.4 Multi-camera (2 or 3 cams), with GUI and RViz

```bash
ros2 launch perception_debug_visualizer debug_visualizer.launch.py \
    camera_config:=$(ros2 pkg prefix perception_bringup)/share/perception_bringup/config/camera_config.yaml \
    active_camera:=0 \
    gui:=true rviz:=true
```

Swap `camera_config.yaml` → `camera_config_2cam.yaml` for two cameras.

### 2.5 Hot-swap active camera at runtime

```bash
ros2 param set /perception_debug_visualizer active_camera_index 1
```

Out-of-range values are rejected; the node logs a `reason` string and keeps
the previous index.

---

## 3. Integrating with the full system launch

`perception_bringup/launch/perception_system.launch.py` already parses
`camera_config`. To run the full pipeline plus the visualizer in one session:

```bash
# Terminal 1 — pipeline
ros2 launch perception_bringup perception_system.launch.py \
    camera_config:=$(ros2 pkg prefix perception_bringup)/share/perception_bringup/config/camera_config_2cam.yaml

# Terminal 2 — visualizer + GUI
ros2 launch perception_debug_visualizer debug_visualizer.launch.py \
    camera_config:=$(ros2 pkg prefix perception_bringup)/share/perception_bringup/config/camera_config_2cam.yaml \
    gui:=true rviz:=true
```

Both launches read the **same** yaml, so namespaces stay consistent end-to-end.

---

## 4. Debugging playbook — symptoms → checks

### 4.1 `/debug/image` has no subscribers / stays black

1. `ros2 topic hz /debug/image` — is the visualizer actually publishing?
   - If no output: the active camera's input image topic is silent. Go to 4.2.
2. `ros2 topic info -v /debug/image` — check QoS. RViz Image display must be
   set to `Reliability Policy: Best Effort` (the preset already is).
3. If using `gui:=true` but the rqt window is blank: `rqt_image_view` defaults
   to reliable QoS. Click the gear icon and switch to `best_effort`.

### 4.2 No input image reaching the node

```bash
ros2 node info /perception_debug_visualizer        # confirm sub topic names
ros2 topic hz /cam0/camera/color/image_raw         # or whichever topic
ros2 topic list | grep image_raw                   # is the driver publishing?
```

- If topic list is empty → RealSense / camera driver not running. See
  [running.md § Camera drivers](running.md#camera-drivers).
- If topic exists but `hz` reports `no new messages`:
  - Check QoS mismatch: `ros2 topic info -v <topic>`. Publisher must be
    `BEST_EFFORT` for the node to receive it (or change both to `RELIABLE`).
  - Check namespace: launch-time `camera_namespaces` vs actual topic prefix.

### 4.3 Image shows, but no bounding boxes

1. `ros2 topic hz /cam0/yolo/detections` — is the tracker publishing?
2. `ros2 topic echo --once /cam0/yolo/detections` — are there detections at
   all? If `detections: []`, it is a YOLO / threshold issue, not a viz issue.
3. Check the camera alignment: the active camera in the visualizer must be the
   same one whose detections you expect. `active_camera_index` selects the
   overlay source.

### 4.4 Mode overlay says `Mode: UNKNOWN`

The meta_controller has not published yet. The visualizer waits for the first
`/meta_controller/active_pipeline` message before filling the label.

```bash
ros2 topic echo --once /meta_controller/active_pipeline
ros2 node list | grep meta_controller
ros2 service call /meta_controller/set_mode perception_msgs/srv/SetMode "{mode: 'NORMAL'}"
```

### 4.5 Smoother poses not showing in RViz

RViz cannot render `perception_msgs/PoseWithMetaArray` directly (custom
message). The bundled preset shows them indirectly via the `object_*_filtered`
TF frames that `pose_filter_cpp` broadcasts. If you don't see those:

```bash
ros2 run tf2_tools view_frames               # generates frames.pdf
ros2 topic echo --once /smoother/smoothed_poses
ros2 topic echo --once /pose_filter/filtered_poses
```

If the smoother is silent but the filter is not, the smoother node probably
didn't start — check `ros2 node list`. Note that `pose_graph_smoother` is a
LifecycleNode and needs to be driven to `ACTIVE` before it publishes.

### 4.6 Multi-camera: merged cloud not showing in RViz

```bash
ros2 topic hz /merged/points
ros2 topic echo --once /merged/points --field header.frame_id
```

- `frame_id` must be `ur5e_base_link` (or whatever `base_frame` in the camera
  config is). If it's a camera frame, the merge TF static transforms are
  missing — rerun calibration / check `static_tf` blocks in
  `camera_config.yaml`.

### 4.7 Cross-camera association drops objects

```bash
ros2 topic echo --once /associated/poses
ros2 param get /cross_camera_associator association_distance_threshold
ros2 param get /cross_camera_associator camera_namespaces
```

- `camera_namespaces` must match **exactly** what the tracker nodes are
  publishing under (the `perception_system.launch.py` pipes the yaml list in
  automatically; confirm they match if you hand-launched).

### 4.8 Phase 4 LifecycleNode stuck in UNCONFIGURED

All Phase 4 launches (`sam2`, `foundationpose`, `cosypose`, `megapose`,
`bundlesdf`) ship with `autostart:=true` by default, which drives the node
`UNCONFIGURED → INACTIVE → ACTIVE` automatically. If the node stays in
`UNCONFIGURED`:

```bash
# Lifecycle state check
ros2 lifecycle get /foundationpose_tracker
ros2 lifecycle nodes                         # all lifecycle nodes in the graph

# If Docker-hosted, tail the entrypoint log to see the configure error
docker compose -f docker/docker-compose.yml logs -f foundationpose
```

Most common causes:
- **Weights / meshes missing** — the `on_configure` callback fails when the
  model directory mount is empty. Confirm the host path (`$FOUNDATIONPOSE_WEIGHTS`
  etc.) exists and the expected files are there. See
  [running.md § Phase 4](running.md#phase-4-ml-nodes-docker).
- **`autostart:=false` was set** explicitly — drive it manually:
  `ros2 lifecycle set /foundationpose_tracker configure` then `activate`.

### 4.9 Phase 4 node runs but host doesn't see its topics

Docker containers use `network_mode: host` + `ipc: host` so DDS traffic
shares the host graph, but the container user needs matching `ROS_DOMAIN_ID`
(and optionally `RMW_IMPLEMENTATION`) with the host:

```bash
# On the host
echo "$ROS_DOMAIN_ID"  "$RMW_IMPLEMENTATION"

# Inside the container
docker exec -it <service> bash -c 'echo "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"'

# From the host, confirm the ML node is actually on the graph
ros2 node list | grep -E "foundationpose|sam2|cosypose|megapose|bundlesdf"
ros2 topic list | grep -E "sam2|foundationpose|cosypose|megapose|bundlesdf"
```

If the lists are empty but `docker compose logs` shows the node spinning,
mismatched `ROS_DOMAIN_ID` is almost always the culprit. See the Docker
gotchas notes in the workspace memory / `docs/installation.md`.

### 4.10 Fan-out launched but only one camera's node appears

All five Phase 4 launches accept `camera_config:=<path>` and spawn one
LifecycleNode per `perception_system.cameras` entry.

```bash
ros2 node list | grep sam2_segmentor              # should list /cam0/sam2_segmentor, /cam1/sam2_segmentor, ...
ros2 topic list | grep -E "/cam[0-9]+/sam2/masks" # one per camera
```

- If only the root-namespace node shows up: confirm `camera_config:=` was
  passed — an empty value triggers the 1-cam fallback path.
- If the namespaces differ from the `perception_system.launch.py` tree, the
  two launches were given different `camera_config` files. Always pass the
  same yaml to both.

---

## 5. Quick diagnostic commands

```bash
# Node + topic graph
ros2 node list
ros2 node info /perception_debug_visualizer
ros2 topic list -t | grep -E "yolo|image|debug|smoother|meta_controller"

# Rates (sanity: real camera ≈ 30 Hz, YOLO ≈ camera_fps, smoother ≈ filter rate)
ros2 topic hz /cam0/camera/color/image_raw
ros2 topic hz /cam0/yolo/detections
ros2 topic hz /debug/image
ros2 topic hz /smoother/smoothed_poses

# Latency (end-to-end: camera → debug image)
ros2 topic delay /debug/image

# Parameters
ros2 param list /perception_debug_visualizer
ros2 param get  /perception_debug_visualizer camera_namespaces
ros2 param get  /perception_debug_visualizer active_camera_index

# Lifecycle (Phase 3 smoother + all Phase 4 ML nodes)
ros2 lifecycle nodes
ros2 lifecycle get /foundationpose_tracker
ros2 lifecycle set /pose_graph_smoother activate

# TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo ur5e_base_link cam0_color_optical_frame

# Docker-hosted Phase 4 nodes
docker compose -f docker/docker-compose.yml ps
docker compose -f docker/docker-compose.yml logs -f <service>
```

---

## 6. Customization pointers (debug visualizer)

| Want to change                 | Edit                                              |
|--------------------------------|---------------------------------------------------|
| Default image topic suffix     | `packages/infrastructure/perception_debug_visualizer/config/visualizer_params.yaml: image_topic_suffix` |
| Detection topic suffix         | `packages/infrastructure/perception_debug_visualizer/config/visualizer_params.yaml: detection_topic_suffix` |
| Bbox / mode font size & color  | `packages/infrastructure/perception_debug_visualizer/src/overlay.cpp` (constants at top of the file) |
| RViz displays                  | `packages/infrastructure/perception_debug_visualizer/rviz/debug_view.rviz` |
| Launch GUI layout              | `packages/infrastructure/perception_debug_visualizer/launch/debug_visualizer.launch.py` |

If you need additional overlays (e.g. smoothed-pose 2D projection, track-id
labels, mask contours from SAM2), extend `detail::draw_*` in
[`overlay.cpp`](../packages/infrastructure/perception_debug_visualizer/src/overlay.cpp)
and call it from `image_callback` in
[`visualizer_node.cpp`](../packages/infrastructure/perception_debug_visualizer/src/visualizer_node.cpp) —
the per-camera cache of the latest message is already available there.

---

## 7. Known limitations of the debug visualizer

- Only one camera's image is republished at a time. Running 3 visualizer
  instances for side-by-side comparison works but consumes 3× CPU. A future
  enhancement could tile N streams into a single image.
- `PoseWithMetaArray` has no RViz plugin. The preset relies on TF frames for
  pose visualization; if you renamed `object_{id}_filtered`, update the TF
  display filter.
- `rqt_image_view` defaults to `RELIABLE` QoS; if the window is blank despite
  `/debug/image` publishing, flip the QoS dropdown in rqt to `best_effort`.
