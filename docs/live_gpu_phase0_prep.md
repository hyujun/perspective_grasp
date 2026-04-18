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

# 모델 저장 루트 (디스크 여유 있는 경로 선택 — 전체 ~30GB)
export MODELS_ROOT=${HOME}/perspective_models   # 또는 /data/perspective_models

# happypose 공유 가중치/메시 (CosyPose + MegaPose)
export HAPPYPOSE_WEIGHTS=${MODELS_ROOT}/happypose
export MEGAPOSE_MESHES=${MODELS_ROOT}/megapose/meshes

# happypose 공식 CLI가 참조하는 변수 — HAPPYPOSE_WEIGHTS와 동일 경로
export HAPPYPOSE_DATA_DIR=${HAPPYPOSE_WEIGHTS}

# FoundationPose
export FOUNDATIONPOSE_WEIGHTS=${MODELS_ROOT}/foundationpose

# BundleSDF
export BUNDLESDF_WEIGHTS=${MODELS_ROOT}/bundlesdf

# (SAM2_WEIGHTS는 repo의 models/sam2/ 기본값 사용 — 설정 불필요)
```

미설정 시 docker-compose는 `<repo>/models/<service>/` fallback. repo의 `models/` 디렉토리에 직접 두는 것도 OK.

---

## 0.6 모델 가중치/메시 다운로드 상세 가이드

각 ML 노드별로 실제 파일을 다운로드하는 절차. 모든 작업은 **위 `0.5`에서 export한 환경변수가 설정된 상태**를 전제로 한다.

### 0.6.0 디렉토리 스켈레톤 생성

다운로드 시작 전 모든 타겟 디렉토리를 미리 만들어두면 이후 경로 실수를 줄일 수 있다.

```bash
mkdir -p "${FOUNDATIONPOSE_WEIGHTS}"/{meshes,weights}
mkdir -p "${HAPPYPOSE_WEIGHTS}"/{bop_datasets,experiments}
mkdir -p "${MEGAPOSE_MESHES}"
mkdir -p "${BUNDLESDF_WEIGHTS}"/{out,weights}

# BundleSDF out 디렉토리는 반드시 쓰기 가능해야 함
chmod -R u+w "${BUNDLESDF_WEIGHTS}/out"
```

### 0.6.1 FoundationPose — refiner + scorer 가중치

FoundationPose 가중치는 NVlabs 공식 Google Drive에서만 제공되며, 두 개의 디렉토리(`2023-10-28-18-33-37` = refiner, `2024-01-11-20-02-45` = scorer) 안에 각각 `model_best.pth`가 들어있다. 이를 받아서 `$FOUNDATIONPOSE_WEIGHTS/weights/` 아래에 **파일명을 리네임**하여 배치해야 한다.

**방법 A — 브라우저로 수동 다운로드 (가장 안정적)**

1. [NVlabs FoundationPose Download weights 링크](https://drive.google.com/drive/folders/1DFezOAD0oD1BblsXVxqDsl8fj0ozB1WU) 접속
2. `2023-10-28-18-33-37/model_best.pth` 다운로드
3. `2024-01-11-20-02-45/model_best.pth` 다운로드
4. 리네임하여 배치:
   ```bash
   cd "${FOUNDATIONPOSE_WEIGHTS}/weights"
   mv ~/Downloads/2023-10-28-18-33-37_model_best.pth ./refiner_model_best.pth
   mv ~/Downloads/2024-01-11-20-02-45_model_best.pth ./scorer_model_best.pth
   ```

**방법 B — gdown CLI (자동화, 쿼터 주의)**

```bash
pip install --user gdown

cd "${FOUNDATIONPOSE_WEIGHTS}/weights"

# Drive 폴더 전체 재귀 다운로드
gdown --folder https://drive.google.com/drive/folders/1DFezOAD0oD1BblsXVxqDsl8fj0ozB1WU \
      -O ./tmp_fp

# 필요한 ckpt만 추출하여 리네임
find ./tmp_fp -name "model_best.pth" | while read f; do
  parent=$(basename "$(dirname "$f")")
  case "$parent" in
    2023-10-28-18-33-37) mv "$f" ./refiner_model_best.pth ;;
    2024-01-11-20-02-45) mv "$f" ./scorer_model_best.pth  ;;
  esac
done
rm -rf ./tmp_fp
```

⚠️ Google Drive는 하루 다운로드 쿼터가 있어 대용량 파일은 종종 실패한다. gdown 실패 시 브라우저에서 "Add shortcut to My Drive" 후 rclone 또는 수동 방식으로 우회.

**메시 준비** — YOLO 클래스명과 1:1 매칭

```bash
cd "${FOUNDATIONPOSE_WEIGHTS}/meshes"

# 예: YCB 메시를 YOLO 클래스명으로 리네임하여 복사
cp /path/to/ycb/002_master_chef_can/textured.obj ./master_chef_can.obj
cp /path/to/ycb/006_mustard_bottle/textured.obj ./mustard_bottle.obj

# 매칭 검증: YOLO data.yaml의 names 리스트와 대조
ls | sed 's/\.[^.]*$//' | sort > /tmp/mesh_names.txt
# YOLO 클래스 리스트와 diff 하여 누락 확인
```

확장자는 `.obj`, `.ply`, `.stl` 모두 허용. 대소문자는 무관하지만 소문자 통일을 권장.

### 0.6.2 CosyPose + MegaPose — happypose CLI 사용

두 패키지는 `happypose` 프레임워크 공식 CLI로 일괄 다운로드. 호스트에 happypose를 설치하면 의존성 지옥(PyTorch/CUDA/Panda3d 충돌)에 빠지기 쉬우므로 **이미 빌드된 `cosypose:latest` 컨테이너 내부에서 실행하는 것이 권장**된다.

**컨테이너 interactive 진입**

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp

docker compose -f docker/docker-compose.yml run --rm \
  -v "${HAPPYPOSE_WEIGHTS}:/ws/models/happypose" \
  -e HAPPYPOSE_DATA_DIR=/ws/models/happypose \
  cosypose bash
```

**컨테이너 내부에서 다운로드**

```bash
# CosyPose YCB-V 모델 (coarse + refiner 필수, detector 선택)
python -m happypose.toolbox.utils.download --cosypose_models \
    detector-bop-ycbv-pbr--970850 \
    coarse-bop-ycbv-pbr--724183 \
    refiner-bop-ycbv-pbr--604090

# MegaPose 전체 모델 (RGB + RGBD variant 모두)
python -m happypose.toolbox.utils.download --megapose_models

# (선택) YCB-V BOP 데이터셋 — 평가/렌더링 검증용, ~15GB
python -m happypose.toolbox.utils.download --bop_dataset ycbv

# (선택) BOP 추가 파일
python -m happypose.toolbox.utils.download --bop_extra_files ycbv
```

**다운로드 명령 상세**

| 명령 플래그 | 다운로드 위치 | 크기 | 필수 여부 |
|---|---|---|---|
| `--cosypose_models detector-bop-ycbv-pbr--970850` | `experiments/detector-bop-ycbv-pbr/` | ~500MB | YOLO 대신 쓸 때만 |
| `--cosypose_models coarse-bop-ycbv-pbr--724183` | `experiments/coarse-bop-ycbv-pbr/` | ~300MB | ✓ 필수 |
| `--cosypose_models refiner-bop-ycbv-pbr--604090` | `experiments/refiner-bop-ycbv-pbr/` | ~300MB | ✓ 필수 |
| `--megapose_models` | `experiments/megapose-models/` (4개 run_id) | ~2GB | MegaPose 사용 시 ✓ |
| `--bop_dataset ycbv` | `bop_datasets/ycbv/` | ~15GB | 평가 시에만 |
| `--bop_extra_files ycbv` | `bop_datasets/ycbv/` | ~100MB | 평가 시에만 |

**MegaPose variant 정리** — `--megapose_models`는 네 가지를 모두 다운로드:

- `coarse-rgb-906902141/` → RGB coarse ✓ 사용 가능
- `refiner-rgb-653307694/` → RGB refiner ✓ 사용 가능
- `coarse-rgbd-.../` → RGBD coarse ✗ 현재 백엔드가 reject (0.4 참조)
- `refiner-rgbd-.../` → RGBD refiner ✗ 현재 백엔드가 reject

디스크 절약을 위해 RGBD 디렉토리 삭제 가능하나, 향후 depth 지원 작업 시 재다운로드 필요하므로 남겨두는 것을 권장.

**메시 준비** — flat 디렉토리, `.obj`/`.ply`만 지원

```bash
cd "${MEGAPOSE_MESHES}"

# YCB 메시 예시 (.stl은 Panda3dBatchRenderer가 silent ignore)
cp /path/to/ycb/006_mustard_bottle/textured.obj ./mustard_bottle.obj
cp /path/to/ycb/002_master_chef_can/textured.obj ./master_chef_can.obj

# FoundationPose 메시와 공유하려면 symlink
ln -sf "${FOUNDATIONPOSE_WEIGHTS}/meshes"/*.obj .
ln -sf "${FOUNDATIONPOSE_WEIGHTS}/meshes"/*.ply .
```

**메시 단위 검증** — mesh_units 설정을 위해 실제 단위 확인

```bash
python3 -c "
import trimesh
m = trimesh.load('${MEGAPOSE_MESHES}/mustard_bottle.obj')
print('extents:', m.extents)
# 값이 ~0.1 수준이면 meters, ~100 수준이면 millimeters
"
```

결과에 따라 `cosypose_params.yaml` / `megapose_params.yaml`의 `mesh_units`를 `"m"` 또는 `"mm"`으로 조정. **단위가 틀리면 추론은 성공해도 pose 위치가 1000배 어긋나 디버깅이 매우 어렵다.**

### 0.6.3 BundleSDF — mesh-free zero-shot tracker

BundleSDF는 mesh-free라 pretrained weights는 필수가 아니다. 기본적으로는 **쓰기 가능한 `out/` 디렉토리만 있으면 동작**한다.

**최소 셋업**

```bash
mkdir -p "${BUNDLESDF_WEIGHTS}/out"
chmod -R u+w "${BUNDLESDF_WEIGHTS}/out"
```

**(선택) LoFTR 가중치** — feature matching용, 컨테이너 빌드에 내장되지 않았을 경우 필요

컨테이너 내부에서 먼저 존재 여부 확인:

```bash
docker compose -f docker/docker-compose.yml run --rm bundlesdf \
  ls /opt/BundleSDF/BundleTrack/LoFTR/weights/
```

`outdoor_ds.ckpt`가 없다면 [LoFTR 공식 Google Drive](https://drive.google.com/drive/folders/1DOcOPZb3-5cWxLqn256AhwUVjBPifhuf)에서 받아 `$BUNDLESDF_WEIGHTS/weights/loftr/outdoor_ds.ckpt`에 배치 후 `docker-compose.yml`의 volume 마운트 추가.

**(선택) XMem 가중치** — segmentation용, 현 파이프라인은 SAM2가 마스크 제공하므로 불필요

필요 시:

```bash
mkdir -p "${BUNDLESDF_WEIGHTS}/weights/xmem"
cd "${BUNDLESDF_WEIGHTS}/weights/xmem"
wget https://github.com/hkchengrex/XMem/releases/download/v1.0/XMem-s012.pth
```

BundleSDF는 메시 파일이 불필요하므로 `meshes/` 디렉토리를 만들지 않는다.

### 0.6.4 SAM2 (참고용 — 이미 live-verified)

이미 셋업된 상태이나 재구성이 필요하면:

```bash
export SAM2_WEIGHTS=${MODELS_ROOT}/sam2
mkdir -p "${SAM2_WEIGHTS}"
cd "${SAM2_WEIGHTS}"

wget https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_large.pt
ls -lh sam2_hiera_large.pt   # ~900MB 확인
```

### 0.6.5 다운로드 검증

모든 다운로드 완료 후 구조 확인:

```bash
# FoundationPose
find "${FOUNDATIONPOSE_WEIGHTS}" -type f | sort
# 기대:
#   .../weights/refiner_model_best.pth
#   .../weights/scorer_model_best.pth
#   .../meshes/<class>.obj (최소 1개)

# happypose
tree -L 3 "${HAPPYPOSE_WEIGHTS}/experiments"
# 기대:
#   coarse-bop-ycbv-pbr/
#     checkpoint.pth.tar
#     config.yaml
#   refiner-bop-ycbv-pbr/
#   megapose-models/
#     coarse-rgb-906902141/
#     refiner-rgb-653307694/

# MegaPose meshes
ls "${MEGAPOSE_MESHES}"
# 기대: <class>.obj 또는 <class>.ply 파일들

# BundleSDF
ls -la "${BUNDLESDF_WEIGHTS}/out"
# 기대: 비어있어도 됨, 단 쓰기 가능
```

### 0.6.6 주의사항 요약

1. **FoundationPose 가중치는 Google Drive 수동 다운로드가 유일한 공식 경로**. 쿼터 제한에 걸리면 하루 대기 필요. gdown 실패 시 브라우저로 우회.
2. **happypose CLI는 반드시 컨테이너 내부에서 실행**. 호스트 설치 시 PyTorch/CUDA/Panda3d 의존성 충돌 발생 가능성 높음.
3. **메시 파일명과 YOLO 클래스명 매칭은 가장 자주 실수하는 포인트**. YOLO 학습 시 `data.yaml`의 `names` 리스트를 확보하여 그 이름 그대로 (소문자, 공백은 underscore) 메시 파일명을 붙일 것.
4. **mesh 단위 확인 누락 시 pose가 1000배 어긋남**. 첫 검증 전 trimesh 등으로 반드시 extents 출력 확인.

---

## 0.7 첫 빌드 캐시 워밍 (선택)

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

## 0.8 검증 시작 전 최종 체크리스트

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
- [ ] `$HAPPYPOSE_WEIGHTS/experiments/coarse-bop-ycbv-pbr/` 존재 (CosyPose)
- [ ] `$HAPPYPOSE_WEIGHTS/experiments/megapose-models/` 존재 (MegaPose)
- [ ] `$MEGAPOSE_MESHES/*.obj` 또는 `*.ply` 존재, mesh_units 단위 확인 완료
- [ ] `$BUNDLESDF_WEIGHTS/out` 쓰기 가능
- [ ] (선택) `docker images | grep perspective_grasp/foundationpose` — 미빌드면 검증 시 ~15분 대기 발생

---

## 0.9 사용자가 사전에 알려줘야 할 정보

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