# Host profiles

Parameter overrides keyed by host class. Each profile lives next to
`camera_config*.yaml` and is consumed by launches that import
`perception_launch_utils.load_host_profile`.

| Profile | Target | Picked when |
|---------|--------|-------------|
| [`dev_8gb.yaml`](./dev_8gb.yaml) | RTX 3070 Ti / similar 8 GB VRAM | `auto` and total VRAM < 12 GB |
| [`prod_16gb.yaml`](./prod_16gb.yaml) | RTX A4000+ / 16 GB+ VRAM | `auto` and total VRAM ≥ 12 GB |
| [`cpu_only.yaml`](./cpu_only.yaml) | No GPU / CI runners | `auto` and `nvidia-smi` not present |

## Selecting a profile

Either via launch arg or env var:

```bash
ros2 launch perception_bringup perception_system.launch.py host_profile:=dev_8gb
# or
PERSPECTIVE_HOST_PROFILE=prod_16gb ros2 launch perception_bringup perception_system.launch.py
```

`auto` (default) picks based on `nvidia-smi --query-gpu=memory.total`. The
chosen profile is logged at launch time.

## YAML shape

```yaml
profile_name: "<id>"
description: "One-line summary"
overrides:
  <node_name>:
    <param>: <value>
    ...
```

`<node_name>` matches the node's `name=` in launch (e.g. `sam2_segmentor`,
`megapose_tracker`, `bundlesdf_tracker`). At launch time the helper
`load_host_profile` returns these as a dict; the launch file passes them
as a trailing entry in `parameters=[base_yaml, profile_overrides]` so they
override the package's default YAML.

## Tradeoffs vs anti-pattern (g)

CLAUDE.md anti-pattern (g) says: do **not** treat 8 GB as a production
limit. These profiles are the legitimate place for that distinction —
they encode "what runs cleanly on which class of host" so the launch
file itself stays branch-free. The architectural rule (one parameterised
code path per multi-camera flow, I4) still applies. Profiles only swap
*parameters*, never code paths.
