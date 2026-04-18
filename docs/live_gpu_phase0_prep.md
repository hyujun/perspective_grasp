# Live GPU Verification — Phase 0 사전 준비

Phase 4 ML 노드 4개 (FoundationPose / CosyPose / MegaPose / BundleSDF) 의 실제 GPU 추론 검증 전에 반드시 마쳐야 하는 환경 준비. SAM2는 이미 live-verified — 이 문서는 나머지 4개에 적용.

전체 검증 플레이북은 `memory/live_gpu_verification_handoff.md` 참조. 이 문서는 그 중 **Phase 0 사전 준비만 상세화**한다.

---

## 0.1 호스트 ROS 2 환경 source

매 새 셸 세션마다:

```bash
cd ~/ros2_ws/perspective_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**왜 필요한가**: 컨테이너 안 ROS 환경은 자체 빌드된 `perception_msgs`로 자족하지만, 호스트에서 `ros2 lifecycle set` / `ros2 topic echo` / `ros2 action send_goal` 같은 검증 명령을 쓰려면 호스트 ROS가 살아있어야 한다.

**워크스페이스 빌드된 적 없으면**:

```bash
./src/perspective_grasp/build.sh
```

---

## 0.2 Docker + GPU 런타임 검증

### nvidia-container-toolkit 동작 확인

```bash
docker run --rm --gpus all nvidia/cuda:12.6.3-base-ubuntu24.04 nvidia-smi
```

- 출력에 GPU 정보 (RTX 3070 Ti, driver 580, etc.) 가 보이면 OK
- 실패하면:
  ```bash
  sudo apt install nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  ```

### CUDA 태그 호환성

`docker_gotchas.md` 메모리 참조 — 사용 중인 베이스 이미지 태그 (`12.6.3-base-ubuntu24.04`) 가 사라질 경우 `12.6.3-runtime-ubuntu24.04` 또는 `12.6.0-base-ubuntu24.04`로 대체 가능. Dockerfile 수정 필요시 `docker/Dockerfile`의 모든 stage `FROM` 라인 동시 변경.

### 실행 중 GPU 프로세스 점검

```bash
nvidia-smi
```

8GB VRAM이 풀로 비어있어야 함. 다른 ML 프로세스 (Ollama, Stable Diffusion, ROS Isaac sim 등) 가 점유 중이면 종료. **이 PC에서는 한 번에 ML 노드 1개만 실행 가능** — 프로덕션 PC는 더 큰 VRAM이라 제약 없음.

---

## 0.3 카메라 + Detection 토픽 살아있는지 확인 ⚠️ 가장 자주 빠뜨리는 단계

Phase 4 컨테이너들은 **호스트 ROS에서 publish되는 토픽을 단순 구독**한다. 컨테이너만 띄운다고 데이터가 나오지 않는다.

### 노드별 필요 토픽

| 노드 | 토픽 | 노트 |
|---|---|---|
| FoundationPose | `/camera/color/image_raw`<br>`/camera/depth/image_rect_raw`<br>`/camera/color/camera_info`<br>`/yolo/detections` | RGB+depth+detections 모두 필수 |
| CosyPose | RGB + camera_info + `/yolo/detections` | depth 불필요 (action 트리거형) |
| MegaPose | RGB + camera_info + `/yolo/detections` | depth 불필요 (RGB-only zero-shot) |
| BundleSDF | RGB + depth + camera_info + **`/sam2/masks`** | YOLO bbox 아님! SAM2 마스크 소비 |
| SAM2 | RGB + camera_info + `/yolo/detections` | (이미 live-verified, 참고용) |

### 호스트 파이프라인 시동

```bash
ros2 launch perception_bringup phase1_bringup.launch.py \
  camera_config:=$(ros2 pkg prefix perception_bringup)/share/perception_bringup/config/camera_config_1cam.yaml
```

이 launch는 RealSense 드라이버 + YOLO tracker를 띄워 위 토픽들을 publish한다.

### Sanity check

```bash
ros2 topic list | grep -E "camera|yolo"
ros2 topic hz /camera/color/image_raw          # ~30Hz
ros2 topic hz /camera/depth/image_rect_raw     # ~30Hz
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /yolo/detections --once        # bbox 한 개라도 나오는지
```

### Multi-camera 사용 시 주의

`camera_config_1cam.yaml`은 namespace 비워둬서 토픽이 정확히 `/camera/...`로 나오고 — phase4 params yaml의 기본값 (`image_topic: "/camera/color/image_raw"` 등) 과 일치한다.

`camera_config_2cam.yaml` / `camera_config.yaml` 사용 시:
- 토픽이 `/cam0/camera/color/image_raw`, `/cam1/...`로 namespace됨
- phase4 params yaml의 `image_topic` 등도 함께 수정 필요
- 또는 `<SERVICE>_CAMERA_CONFIG` 환경변수로 launch 시점에 fan-out 모드 활성화

---

## 0.4 모델 가중치 + 메시 디렉토리 레이아웃

각 서비스는 docker-compose volume 마운트로 컨테이너 내부 `/ws/models/...`에 매핑된다. 호스트 측 경로는 환경변수로 지정.

### FoundationPose

**환경변수**: `$FOUNDATIONPOSE_WEIGHTS` (기본: `<repo>/models/foundationpose/`)
**컨테이너 마운트**: `→ /ws/models/foundationpose`

```
foundationpose/
├── meshes/
│   ├── <yolo_class_name>.obj    # YOLO 클래스명 = 파일명 (소문자, 확장자 무관)
│   ├── <yolo_class_name>.ply    # .obj/.ply/.stl 모두 OK (대소문자 무관)
│   └── ...
└── weights/
    ├── refiner_model_best.pth   # NVIDIA FoundationPose refiner ckpt
    └── scorer_model_best.pth    # NVIDIA FoundationPose scorer ckpt
```

🔑 **메시 파일명 = YOLO 클래스명**. YOLO가 `bottle`로 detect하면 `meshes/bottle.obj`가 있어야 매칭. 매칭 안 되면 silent skip (debug 레벨 로그).

📎 [FoundationPose 가중치 다운로드 가이드](https://github.com/NVlabs/FoundationPose#download-weights)

### CosyPose + MegaPose (가중치/메시 공유)

**환경변수**:
- `$HAPPYPOSE_WEIGHTS` (기본: `<repo>/models/happypose/`) → `/ws/models/happypose`
- `$MEGAPOSE_MESHES` (기본: `<repo>/models/megapose/meshes/`) → `/ws/models/megapose/meshes`

```
happypose/                          # = HAPPYPOSE_DATA_DIR
├── bop_datasets/
│   └── ycbv/                       # cosypose_params.yaml dataset_name 기본값
│       ├── models/
│       └── ...
└── experiments/
    ├── coarse-bop-ycbv-pbr/        # cosypose_params.yaml coarse_run_id 비워두면 이 이름으로 자동 검색
    │   ├── checkpoint.pth.tar
    │   └── config.yaml
    ├── refiner-bop-ycbv-pbr/
    │   ├── checkpoint.pth.tar
    │   └── config.yaml
    └── megapose-models/            # MegaPose는 이 하위에서 NAMED_MODELS의 run_id 검색
        ├── coarse-rgb-906902141/
        └── refiner-rgb-653307694/

megapose/meshes/                    # 평면 디렉토리, CosyPose+MegaPose 공유
├── <yolo_class>.obj                # .obj/.ply만 (Panda3dBatchRenderer 제약)
├── <yolo_class>.ply                # .stl 미지원 — silent ignore됨
└── ...
```

🔑 **mesh_units 확인**: `cosypose_params.yaml` / `megapose_params.yaml`의 `mesh_units: "mm"` 기본값. 메시가 미터 단위면 `"m"`으로 바꿔야 한다. 단위 틀리면 추론은 돌지만 위치가 1000배씩 어긋난다.

🔑 **MegaPose 모델 선택 제약** (2026-04-18 추가):
- `model_name: "megapose-1.0-RGB"`, `"megapose-1.0-RGB-multi-hypothesis"` ✓ 사용 가능
- `"megapose-1.0-RGBD"`, `"megapose-1.0-RGB-multi-hypothesis-icp"` ✗ — `requires_depth=True` 라서 백엔드가 `load()` 시점에 명시적으로 reject. depth_topic 추가 작업 후에야 사용 가능.

📎 [happypose 가중치 다운로드](https://agimus-project.github.io/happypose/cosypose/download_data.html)
```bash
# 컨테이너 내부에서 실행 권장
python -m happypose.toolbox.utils.download --cosypose_models \
  detector-bop-ycbv-pbr--970850 \
  coarse-bop-ycbv-pbr--724183 \
  refiner-bop-ycbv-pbr--604090
```

### SAM2 (이미 셋업됨)

**환경변수**: `$SAM2_WEIGHTS` (기본: `<repo>/models/sam2/`) → `/ws/models/sam2`

```
sam2/
└── sam2_hiera_large.pt            # ✓ 이미 존재 (live-verified)
```

추가 작업 불필요. BundleSDF가 SAM2 마스크를 소비하므로 BundleSDF 검증 시 SAM2 컨테이너도 동시 실행 필요.

### BundleSDF

**환경변수**: `$BUNDLESDF_WEIGHTS` (기본: `<repo>/models/bundlesdf/`) → `/ws/models/bundlesdf`

```
bundlesdf/
├── out/                           # 쓰기 가능 — per-track SDF + debug 출력 저장됨
└── weights/                       # 선택 — 사전학습 NeRF / scorer 가중치 (있으면 추가)
```

BundleTrack/NeRF 설정 파일은 컨테이너 내부 `/opt/BundleSDF/...` 사용 (clone된 NVlabs 리포). `bundlesdf_params.yaml`의 `cfg_track_path`, `cfg_nerf_path` 가 이를 참조.

📎 [BundleSDF setup](https://github.com/NVlabs/BundleSDF) — pretrained weights는 선택, mesh-free zero-shot tracker라 필수 아님.

---

## 0.5 환경변수 영구 설정

매번 export하기 귀찮으면 `~/.bashrc` 또는 별도 sourceable 스크립트:

```bash
# ~/.bashrc 또는 ~/perspective_grasp_env.sh

# happypose 공유 가중치/메시 (CosyPose + MegaPose)
export HAPPYPOSE_WEIGHTS=/path/to/models/happypose
export MEGAPOSE_MESHES=/path/to/models/megapose/meshes

# FoundationPose
export FOUNDATIONPOSE_WEIGHTS=/path/to/models/foundationpose

# BundleSDF
export BUNDLESDF_WEIGHTS=/path/to/models/bundlesdf

# (SAM2_WEIGHTS는 repo의 models/sam2/ 기본값 사용 — 설정 불필요)
```

미설정 시 docker-compose는 `<repo>/models/<service>/` fallback. repo의 `models/` 디렉토리에 직접 두는 것도 OK.

---

## 0.6 첫 빌드 캐시 워밍 (선택)

빌드 안 된 이미지를 사전에 빌드해두면 검증 시 대기 시간 단축:

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp

# 현재 빌드 상태
docker images | grep perspective_grasp
# perspective_grasp/cosypose:latest    ✓ (25.8GB)
# perspective_grasp/sam2:latest         ✓ (29GB)
# perspective_grasp/foundationpose      ✗ 미빌드
# perspective_grasp/bundlesdf           ✗ 미빌드

# 백그라운드에서 미리 빌드 (~15분 each on RTX 3070 Ti)
docker compose -f docker/docker-compose.yml build foundationpose &
docker compose -f docker/docker-compose.yml build bundlesdf &
wait
```

⚠️ 두 개 동시 빌드 시 디스크/네트워크 부하 큼 — 순차 빌드 권장.

⚠️ kaolin wheel URL이 404날 수 있음 (`docker_gotchas.md` 참조). `Dockerfile`에서 `https://nvidia-kaolin.s3.us-east-2.amazonaws.com/torch-2.6.0_cu126.html` 가 dead면 현재 릴리스 URL로 교체.

---

## 0.7 검증 시작 전 최종 체크리스트

Phase 1 (FoundationPose) 들어가기 전 모두 ✓ 되어야 함.

- [ ] `source install/setup.bash` 완료, `ros2` 명령 동작
- [ ] `docker run --rm --gpus all nvidia/cuda:12.6.3-base-ubuntu24.04 nvidia-smi` 정상 출력
- [ ] `nvidia-smi`로 8GB VRAM 비어있음 확인 (다른 GPU 프로세스 없음)
- [ ] 호스트 카메라 파이프라인 가동 중 (`phase1_bringup.launch.py`)
- [ ] `ros2 topic list`에 `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/color/camera_info`, `/yolo/detections` 모두 존재
- [ ] `ros2 topic hz /camera/color/image_raw` ~30Hz 출력
- [ ] `ros2 topic echo /yolo/detections --once` — bbox 한 개라도 나옴
- [ ] `$FOUNDATIONPOSE_WEIGHTS` 가리키는 디렉토리 존재, `meshes/`와 `weights/` 둘 다 채워짐
- [ ] `meshes/` 안 파일명이 YOLO 클래스명과 매칭 (소문자, 적어도 1개)
- [ ] (선택) `docker images | grep perspective_grasp/foundationpose` — 미빌드면 검증 시 ~15분 대기 발생

---

## 0.8 사용자가 사전에 알려줘야 할 정보

본 문서를 참고해 검증을 시작할 다음 세션에:

1. **모델 가중치 위치** (4개 경로):
   - FoundationPose refiner+scorer ckpt + per-class meshes
   - happypose experiments + bop_datasets
   - megapose meshes (flat dir)
   - bundlesdf out 디렉토리 (writable)
2. **YOLO 클래스명 리스트** — 메시 파일명과 1:1 매칭 검증용
3. **메시 단위** (mm vs m) — `mesh_units` yaml 수정 여부 결정
4. **카메라 셋업**:
   - RealSense 연결됨? 모델 (D435i, D455 등)
   - 1cam? 2cam? 3cam?
   - Eye-in-hand vs eye-to-hand?

이 4가지가 결정되면 Phase 1 (FoundationPose) live verification 즉시 시작 가능.

---

## 관련 문서

- 전체 검증 플레이북: `memory/live_gpu_verification_handoff.md`
- happypose 설치 함정: `memory/happypose_install_trap.md`
- Docker 빌드 함정: `memory/docker_gotchas.md`
- CosyPose/MegaPose ship 상태: `memory/cosypose_megapose_shipped.md`
- BundleSDF ship 상태: `memory/bundlesdf_shipped.md`
- FoundationPose ship 상태: `memory/foundationpose_shipped.md`
