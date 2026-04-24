# perspective_grasp — Claude Harness

Harness for Claude working in this repo. Start here before editing. Section 2 (Invariants),
4 (Sensors), and 5 (Escalation Triggers) are the load-bearing parts — if a change looks like
it crosses any of those, stop and re-read.

## 1. Project Snapshot

RGB-D 6D pose estimation pipeline + UR5e + 10-DoF hand manipulation. **18 ROS 2 packages** in
`packages/<group>/<pkg>/` (colcon discovers recursively), 5 phases + bringup + infrastructure.

- Architecture principle: **Vision Push, Controller Pull.** Vision broadcasts TF2 continuously;
  the controller does `lookupTransform()` at its own rate. Long ops use ROS 2 Action Servers.
- Host + Docker hybrid: C++ nodes and camera drivers on host; GPU-heavy Python ML (Phase 4) in
  Docker containers sharing `network_mode: host` + `ipc: host` with the host DDS.
- Phase 4 is code-complete: all 5 ML nodes ship pluggable real+mock backends, dedicated Docker
  stages, and multi-camera fan-out. SAM2 is live-verified on hardware; FoundationPose / CosyPose /
  MegaPose / BundleSDF are mock-smoke-tested (live GPU inference pending user-side images + weights).
- Phase 5 `grasp_pose_planner` antipodal planner + `Hand10DoF` adapter are implemented. Real
  10-DoF preshape joint mapping is a TODO (see §5 Escalation Triggers).

References:
- Full architecture, topics, TF frames, QoS, pipeline modes, optional deps: [docs/architecture.md](./docs/architecture.md)
- Install: [docs/installation.md](./docs/installation.md) — Building: [docs/build.md](./docs/build.md) — Running: [docs/running.md](./docs/running.md) — Debugging: [docs/debugging.md](./docs/debugging.md)
- Live-GPU prep playbook: [docs/live_gpu_phase0_prep.md](./docs/live_gpu_phase0_prep.md)
- Per-package detail: `packages/<group>/<pkg>/README.md`

## 2. Repository Invariants

Violating any of these breaks the architecture. If a change looks like it *must* touch one of
these, **stop and ask** — do not "try harder" to make it work.

| # | Invariant | Why | Grep-detectable violation signal |
|---|-----------|-----|----------------------------------|
| I1 | Vision Push, Controller Pull | Control loop must not block on perception callbacks | `wait_for_message`, `spin_until_future_complete` inside a control-rate timer callback |
| I2 | Heavy ops = Action Server, not Service | Services block callers; actions support cancel/feedback | New long-running `.srv` for scene analysis / grasp planning / scan-style ops |
| I3 | No code coupling with `ur5e_ws/` | Cross-workspace boundary is TF2 frames + action interfaces only | `find_package(ur5e_*)`, `import ur5e_*`, `#include <ur5e_...>` anywhere under `packages/` |
| I4 | Multi-camera path is unified | N=1 is "one-camera multi-camera" — single parameterized code path | `if num_cameras == 1`, `if N == 1`, `if len(cameras) == 1` in launch / fusion code |
| I5 | Optional deps have fallbacks | TEASER++, Ceres, GTSAM must be skippable without breaking build/run | `find_package(<opt> REQUIRED)` for TEASER++ / Ceres / GTSAM; missing `if(<opt>_FOUND)` guard |
| I6 | `perception_msgs` is the only shared interface | Cross-workspace ABI — silent drift breaks `ur5e_ws/` consumers | Field added/renamed/removed in `.msg`/`.srv`/`.action` without a `docker compose build` + consumer audit |
| I7 | Control-path QoS = BEST_EFFORT + depth 1 | Stale pose data is worse than none | `RELIABLE` or `KEEP_ALL` on `/pose_filter/*`, `/smoother/*`, `/associated/*`, `/merged/*` |

## 3. Workflow Loop (the harness)

Run this loop per task. Skip a step only when noted. **When a step fails, diagnose — don't
retry blindly. Add a test / guard / fallback that encodes the missing constraint, or escalate.**

1. **Locate** — known symbol/file → `grep` / `glob`. Broad "where is X used across 18 packages"
   → `Agent` with `Explore` subagent.
2. **Read** — target file + its `package.xml` + `CMakeLists.txt`. **Check: is this host C++ or
   Docker Python?** (Phase 4 nodes run inside `ml-base` container — different build surface.)
3. **Edit** — minimal change, single concern. Identifiers in English; comments only when the
   *why* is non-obvious (CLAUDE system prompt rule). Follow existing file style.
4. **Build** — `colcon build --packages-select <pkg>` first. Use `./build.sh` only when
   dependency order matters (editing `perception_msgs` or `teaser_icp_hybrid_registrator`).
5. **Test** — `colcon test --packages-select <pkg>` + `colcon test-result --verbose`. If you
   fixed a bug, add a regression test before shipping.
6. **Smoke** — minimum launch needed to exercise the change. Watch for QoS warnings, TF
   timeouts, unhandled exceptions. UI-less? Say so and hand off to the user. Don't claim
   "works" on the strength of unit tests alone.
7. **Docs sync (mandatory before any commit)** — re-read your own diff, then update *every*
   documentation surface the change invalidates. In order:
   - **Touched package's `README.md`** under `packages/<group>/<pkg>/README.md` — topics,
     parameters, test counts/binaries, launch names, lifecycle states, dependency notes.
   - **Root [`README.md`](./README.md)** — if package count, top-level capabilities, install
     path, or quick-start changed.
   - **[`docs/`](./docs/) files** — architecture (`architecture.md` for any topic/TF/QoS/
     package/mode change), build/install/running/debugging for command or flow changes, and
     `live_gpu_phase0_prep.md` for Phase 4 weights/meshes/Docker changes.
   - **This `CLAUDE.md`** — if an invariant, sensor, escalation trigger, or anti-pattern
     became true or stale.

   Treat "docs unchanged" as a claim you must justify, not a default. If nothing needs
   updating, say so out loud in the commit/PR message. Never commit code + docs drift in
   the same session hoping to fix it later.

## 4. Sensors (verification surface)

What to run to confirm a change is OK. Per change-type:

| Change type | Sensor(s) |
|-------------|-----------|
| C++ code in any package | `colcon build --packages-select <pkg>` → `colcon test --packages-select <pkg>` → `colcon test-result --verbose` |
| Python node (Phase 4 or bringup helper) | Package's own `pytest` suite (e.g. `perception_launch_utils`). **TODO: no repo-wide ruff/mypy config — add when lint infra lands.** |
| `perception_msgs` .msg/.srv/.action | Above **plus** `COMPOSE_BAKE=true docker compose -f docker/docker-compose.yml build` **plus** rebuild every downstream consumer (host + `ur5e_ws/`) |
| Launch / config YAML | `ros2 launch <pkg> <launch.py> --print` dry-run + short smoke run + `ros2 topic list` sanity |
| Filter / estimator parameters (pose_filter_cpp, pose_graph_smoother) | Existing gtests cover logic. **TODO: no bag-based regression harness — parameter tuning is currently visual-inspection only.** |
| Docker image content | `docker compose build <service>` then `docker compose run --rm <service> ros2 node list` |
| **Any commit** | Manual docs audit per §3 step 7: touched `packages/<...>/README.md`, root [`README.md`](./README.md), relevant [`docs/*.md`](./docs/), and this `CLAUDE.md`. `git diff --stat` + `git diff -- '*.md'` before `git commit`. |

Rough signal: if all relevant sensors in the matched row are green, the change is probably
safe to stop on. If a sensor is TODO-marked, say so when reporting completion — don't silently
skip verification.

## 5. Escalation Triggers

Stop and check with the user before proceeding when any of these apply.

| Situation | Why |
|-----------|-----|
| `perception_msgs` modification needed | I6 — cross-workspace ABI; `ur5e_ws/` consumers must be rebuilt in lockstep |
| New `ur5e_ws/` code dep seems needed | I3 — see if TF2/action routing can solve it first |
| User asks to "implement Phase 4 node X" | All 5 already ship; clarify intent — tuning? live-run help? new feature on top? |
| Optional dep's fallback needs removing | I5 — either commit to the hard dep (with install-script change) or keep fallback |
| Code-path needs to branch on camera count | I4 — the architecture says one path parameterized by config; ask why the config path fails |
| Hand10DoF finger-joint preshape mapping (Phase 5) | Depends on the physical hand's kinematic model — need concrete spec before coding |
| New CUDA / kernel module / GPU runtime dep | Affects both host install scripts and Docker base image; user may want to defer |
| Phase 0 discrepancy: CLAUDE.md/memory says X but code shows Y | Ask which is authoritative before acting; update the wrong one |

## 6. Anti-patterns (real failure modes)

Past mistakes, catalogued so they stop recurring. Each entry: **symptom → cause → detection →
recovery.**

a. **"It's already implemented" oversight** — Symptom: agent re-implements a Phase 4 node from
   scratch. Cause: request phrased as "implement X" obscures that node X already ships with
   real+mock backends. Detection: check `packages/phase4_refinement/<node>/` before writing
   new code. Recovery: stop, ask whether the user wants tuning, live-run support, or a new
   feature *on top* of the existing node.

b. **Silent ABI skew after `perception_msgs` edit** — Symptom: host publisher and container
   subscriber disagree on a field, topic looks connected but messages are dropped. Cause:
   `docker compose build` skipped after editing `.msg`/`.srv`/`.action`. Detection: after
   any `.msg`/`.srv`/`.action` change, you **must** run `docker compose build`; audit `ur5e_ws/`
   consumers too (I6). Recovery: rebuild `msgs-builder` stage + every runtime stage.

c. **QoS-mismatch silent disconnect** — Symptom: topic is listed by `ros2 topic list` but
   `ros2 topic echo` shows nothing. Cause: publisher `RELIABLE` + subscriber `BEST_EFFORT` (or
   vice versa). Detection: `ros2 topic info -v <topic>` shows mismatched QoS. Recovery: force
   control-path topics to BEST_EFFORT + depth 1 (I7); ask the user before deviating.

d. **`if N == 1:` branch sneaking into multi-camera path** — Symptom: 1-cam works, 2-cam
   regresses (or vice versa). Cause: agent added a single-camera fast path. Detection:
   `grep -rE 'num_cameras ?== ?1|len\(cameras\) ?== ?1' packages/{bringup,phase*,infrastructure}`.
   Recovery: delete the branch; use the existing `camera_config*.yaml`-driven path (I4).

e. **Hardcoded package counts in docs/scripts** — Symptom: CLAUDE.md / README / install script
   cites "17 packages" while tree has 18 (git log: `1e3c58b` refresh). Cause: number pasted
   into prose instead of computed. Detection: search for `\b1[0-9]\s*package` in docs/scripts.
   Recovery: prefer `find packages -name package.xml | wc -l` at verification time over a
   stored count.

f. **`pose_graph_smoother` frame_id overwrite** — Symptom: RViz TF tree breaks after enabling
   smoother in passthrough mode. Cause: agent applied `fallback_parent_frame` unconditionally.
   Detection: `test_smoother_node_smoke` covers this — do not delete. Recovery: passthrough
   must preserve upstream frame_id; only apply fallback when input frame_id is empty.

g. **Misreading 8 GB VRAM as a production constraint** — Symptom: agent refuses to wire two
   ML nodes together "because VRAM won't fit." Cause: conflating dev-machine (RTX 3070 Ti
   8 GB) with production (different, higher-spec PC). Detection: any architectural argument
   that cites the 8 GB number. Recovery: clarify dev vs prod before arguing; mode switching
   is for dev ergonomics, not a production hard limit.

h. **Unthrottled logging in a real-time callback** — Symptom: per-frame `RCLCPP_INFO`
   in a YOLO/ICP/filter loop drags latency up and buries real warnings. Cause: debug logging
   left in place. Detection: `grep -rE 'RCLCPP_(INFO|WARN|ERROR)\(' packages/ | grep -v THROTTLE`
   inside callback scopes. Recovery: use `RCLCPP_*_THROTTLE(this->get_logger(), *get_clock(), ms, ...)`.

i. **`pose_filter_cpp` Q-scaling unit drift** — Symptom: IEKF diverges or underweights
   process noise with non-nominal `dt`. Cause: Q treated as per-step instead of per-second
   (git log: `a8f21f2`). Detection: tests `test_iekf_se3` encode the fix — don't weaken them.
   Recovery: Q ∝ dt; keep units consistent with the spec in [pose_filter_cpp README](./packages/phase3_filtering/pose_filter_cpp/README.md).

j. **Phase 4 LifecycleNode stuck UNCONFIGURED** — Symptom: Docker service "up" but no topics
   appear. Cause: entrypoint didn't drive `configure`→`activate` (git log: `4fcd23a`).
   Detection: `ros2 lifecycle get /<ns>/<node>` shows `unconfigured`. Recovery: use the
   auto-activate entrypoint; don't require the user to `ros2 lifecycle set` manually.

k. **`rclpy.shutdown()` double-call** — Symptom: Python node crashes on Ctrl-C with "already
   shutdown." Cause: agent added `rclpy.shutdown()` in both `finally` and a signal handler
   (git log: `c47f504`). Detection: `grep -rE 'rclpy\.shutdown\(\)' packages/`. Recovery: use
   `rclpy.try_shutdown()` — idempotent, safe to call multiple times.

l. **Robot-specific frame names in `camera_config.yaml`** — Symptom: `camera_config.yaml`
   carries `ur5e_base_link` / `ur5e_tool0`, blocking reuse on other arms. Cause: vision config
   leaked controller-specific frames (git log: `cd6633a`). Detection: `grep -E 'ur5e_|(^|\s)base_link' packages/bringup/perception_bringup/config/*.yaml`.
   Recovery: keep robot-agnostic names (`base`, `tool0`) in camera_config; the static TF from
   `ur5e_base_link` is applied elsewhere.

## 7. Where Things Live (mental map)

Detailed package table: [docs/architecture.md#packages-18-total](./docs/architecture.md#packages-18-total).
Quick mental map for navigation:

- `packages/interfaces/perception_msgs/` — every message / service / action. Changes here
  cascade to `ur5e_ws/` and Docker images (I6).
- `packages/bringup/perception_bringup/` — system launches (`perception_system.launch.py`,
  `phase1_bringup.launch.py`) + `config/camera_config*.yaml`. Start here when hunting
  "how does the pipeline wire together."
- `packages/phase1_perception/` — YOLO tracker + TEASER/ICP registrator (library target).
- `packages/phase2_fusion/` — cross-camera associator (Hungarian + Union-Find) + PCL merge.
- `packages/phase3_filtering/` — SE(3) IEKF pose filter + optional GTSAM pose-graph smoother.
- `packages/phase4_refinement/` — SAM2 / FoundationPose / CosyPose / MegaPose / BundleSDF.
  **All run in Docker**, not on host. See `docker/docker-compose.yml` for service names.
- `packages/phase5_manipulation/grasp_pose_planner/` — antipodal planner + `Hand10DoF`
  adapter (preshape mapping TODO, see §5).
- `packages/infrastructure/` — `perception_meta_controller`, `perception_debug_visualizer`,
  `multi_camera_calibration`, `perception_launch_utils` (shared Python launch helpers).

## 8. Common Commands

```bash
# Single-package iteration (preferred)
cd /home/junho/ros2_ws/perspective_ws
colcon build --packages-select <pkg>
colcon test --packages-select <pkg> && colcon test-result --verbose
source install/setup.bash

# Full build (only when dependency order matters)
./src/perspective_grasp/build.sh               # RelWithDebInfo (default)
./src/perspective_grasp/build.sh --clean       # Wipe build/install/log first

# Phase 4 Docker images
COMPOSE_BAKE=true docker compose -f docker/docker-compose.yml build
```

Build internals, test breakdown by package, Release toggles, Docker rebuild recipes:
[docs/build.md](./docs/build.md). Running / launching: [docs/running.md](./docs/running.md).

## 9. Style Cheatsheet

- C++20 (`cxx_std_20`) with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`; `ament_cmake`.
- Python: `ament_cmake_python` for mixed packages, else `ament_python`.
- Formatting: clang-format (Google style with project modifications).
- Namespace: `perspective_grasp`.
- **Launch files**: import from `perception_launch_utils` — `config_path`, `share_file`,
  `load_config`, `declare_{params_file,camera_config,autostart}_arg`, `fanout_lifecycle_nodes`.
  Do **not** hand-roll `os.path.join(get_package_share_directory(...), 'config', ...)` or YAML
  parsing. Add `<exec_depend>perception_launch_utils</exec_depend>` to `package.xml`.
- Comments default to none; add only when the *why* is non-obvious (hidden constraint, subtle
  invariant, workaround). Don't narrate *what* well-named code already says.

## 10. Claude-specific Notes

- **Verify before trusting memory.** Memory files under [memory/](../../../../.claude/projects/-home-junho-ros2-ws-perspective-ws-src-perspective-grasp/memory/)
  are point-in-time snapshots. Before citing counts, paths, or dep status from memory, verify
  with `ls` / `grep` / `git log`. (Absolute path: `/home/junho/.claude/projects/-home-junho-ros2-ws-perspective-ws-src-perspective-grasp/memory/`.)
- **Subagents**: use `Agent` with `subagent_type=Explore` for broad "where is X used" across
  18 packages. Direct `Grep`/`Glob` is fine for known symbols.
- **Related skills**: `init` / `claude-md-improver` (maintain this file), `review` /
  `security-review` (PR review), `simplify` (code review pass), `claude-api` (only if a node
  starts calling hosted Claude models — none do today).
- **User language**: user mixes Korean and English. Mirror the user's language in prose; keep
  technical identifiers (topic names, class names, CLI flags) in English regardless.

## 11. Workspace Paths

- This repo: `/home/junho/ros2_ws/perspective_ws/src/perspective_grasp/`
- Controller (separate workspace — no code coupling, see I3): `/home/junho/ros2_ws/ur5e_ws/src/ur5e-rt-controller/`
- ROS 2: `/opt/ros/jazzy/`
- Memory: `/home/junho/.claude/projects/-home-junho-ros2-ws-perspective-ws-src-perspective-grasp/memory/`
