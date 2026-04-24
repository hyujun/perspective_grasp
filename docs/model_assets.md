# Phase 4 Model Weights & Meshes

5개 Phase 4 ML 노드 (SAM2 / FoundationPose / CosyPose / MegaPose / BundleSDF) 가 컨테이너 내부에서 기대하는 **가중치 + 메시 디렉토리 구성**과 **호스트에서 받는 방법**을 다룬다.

이 문서가 다루지 **않는** 것 (다른 문서 참조):

| 주제 | 문서 |
|---|---|
| OS / 드라이버 / ROS 2 / Docker / venv 설치 | [installation.md](./installation.md) |
| `models/<service>/` 디렉토리 스켈레톤 자동 생성 | [installation.md](./installation.md) `install_host.sh` step 7 |
| RealSense / YOLO / phase1 launch 절차 | [running.md](./running.md) |
| Docker 이미지 빌드 / 재빌드 / 캐시 무효화 | [build.md](./build.md) §Rebuilding Docker images |
| 컨테이너 lifecycle 구동 + API drift 대응 | `memory/live_gpu_verification_handoff.md` |
| 토픽이 안 보일 때 디버깅 | [debugging.md](./debugging.md) §4.8–4.10 |

전제: `install_host.sh` 가 이미 실행되어 `<repo>/models/{foundationpose,happypose,megapose/meshes,sam2,bundlesdf}/` 가 존재한다. 다른 디스크에 두려면 §3 환경변수.

---

## 1. 컨테이너 마운트 계약

각 서비스는 `docker/docker-compose.yml`의 volume 마운트로 호스트 디렉토리를 컨테이너 내부 `/ws/models/...`에 매핑한다. params yaml은 컨테이너 경로를 하드코드.

| 서비스 | 호스트 env var (default) | 컨테이너 마운트 | params yaml의 하드코드 경로 |
|---|---|---|---|
| sam2 | `$SAM2_WEIGHTS` (`<repo>/models/sam2`) | `/ws/models/sam2` | `model_checkpoint: /ws/models/sam2/sam2_hiera_large.pt`<br>`model_config: sam2_hiera_l.yaml` |
| foundationpose | `$FOUNDATIONPOSE_WEIGHTS` (`<repo>/models/foundationpose`) | `/ws/models/foundationpose` | `mesh_dir: /ws/models/foundationpose/meshes`<br>(weights 관례: `/ws/models/foundationpose/weights/`) |
| cosypose | `$HAPPYPOSE_WEIGHTS` (`<repo>/models/happypose`)<br>`$MEGAPOSE_MESHES` (`<repo>/models/megapose/meshes`) | `/ws/models/happypose`<br>`/ws/models/megapose/meshes` | `dataset_dir: /ws/models/happypose`<br>`mesh_dir: /ws/models/megapose/meshes` |
| megapose | (cosypose와 동일 — 같은 이미지/마운트 공유) | 동일 | 동일 |
| bundlesdf | `$BUNDLESDF_WEIGHTS` (`<repo>/models/bundlesdf`) | `/ws/models/bundlesdf` | `out_dir: /ws/models/bundlesdf/out`<br>(BundleTrack/NeRF cfg는 컨테이너 안 `/opt/BundleSDF/`) |

---

## 2. 디렉토리 레이아웃

### SAM2

```
models/sam2/
└── sam2_hiera_large.pt           # ~900MB, Meta FAIR 공식
```

파일명 고정 — `sam2_params.yaml`이 정확히 이 경로를 본다. 다른 모델 크기 (tiny/small/base_plus) 를 쓰려면 `model_checkpoint` + `model_config` 둘 다 yaml에서 변경. 현재 pin된 SAM2 SHA (`2b90b9f5`, 2024-12) 는 SAM 2.0 API만 지원 — 2.1 (`sam2.1_hiera_*`) 을 쓰려면 backend + Dockerfile pin 변경 필요.

### FoundationPose

```
models/foundationpose/
├── meshes/
│   ├── <yolo_class_name>.obj    # YOLO 클래스명 = 파일명 (소문자, 확장자 무관)
│   ├── <yolo_class_name>.ply    # .obj / .ply / .stl 모두 OK
│   └── ...
└── weights/
    ├── refiner_model_best.pth   # NVIDIA FoundationPose refiner ckpt
    └── scorer_model_best.pth    # NVIDIA FoundationPose scorer ckpt
```

🔑 **메시 파일명 = YOLO 클래스명**. YOLO가 `bottle`로 detect하면 `meshes/bottle.obj`가 있어야 매칭. 매칭 안 되면 silent skip (debug 레벨 로그).

### CosyPose + MegaPose (가중치/메시 공유)

```
models/happypose/                  # = HAPPYPOSE_DATA_DIR (컨테이너 env)
├── bop_datasets/
│   └── ycbv/                      # cosypose_params.yaml dataset_name 기본값
│       ├── models/
│       └── ...
└── experiments/
    ├── coarse-bop-ycbv-pbr/       # CosyPose coarse
    │   ├── checkpoint.pth.tar
    │   └── config.yaml
    ├── refiner-bop-ycbv-pbr/      # CosyPose refiner
    │   ├── checkpoint.pth.tar
    │   └── config.yaml
    └── megapose-models/           # MegaPose는 이 하위에서 NAMED_MODELS의 run_id 검색
        ├── coarse-rgb-906902141/
        └── refiner-rgb-653307694/

models/megapose/meshes/             # 평면 디렉토리, CosyPose+MegaPose 공유
├── <yolo_class>.obj                # .obj/.ply만 (Panda3dBatchRenderer 제약)
├── <yolo_class>.ply                # .stl 미지원 — silent ignore됨
└── ...
```

🔑 **mesh_units 확인** (`cosypose_params.yaml` / `megapose_params.yaml`): 기본 `mesh_units: "mm"`. 메시가 미터 단위면 `"m"`으로 변경. **단위 틀리면 추론은 돌지만 pose 위치가 1000배씩 어긋난다.**

🔑 **MegaPose 모델 선택 제약**:
- `model_name: "megapose-1.0-RGB"`, `"megapose-1.0-RGB-multi-hypothesis"` ✓ 사용 가능
- `"megapose-1.0-RGBD"`, `"megapose-1.0-RGB-multi-hypothesis-icp"` ✗ — `requires_depth=True` 라서 백엔드가 `load()` 시점에 reject. depth_topic 배선 후에야 사용 가능.

### BundleSDF

```
models/bundlesdf/
├── out/                           # 쓰기 가능 — per-track SDF + debug 출력 저장됨
└── weights/                       # 선택 — LoFTR / XMem 등 (있으면 volume mount 추가 필요)
```

BundleSDF는 mesh-free zero-shot tracker라 `meshes/` 불필요. BundleTrack/NeRF 설정 파일은 컨테이너 안 `/opt/BundleSDF/` (clone된 NVlabs 리포) 에서 읽으므로 호스트에서 건드릴 필요 없음.

---

## 3. 환경변수 영구 설정 (선택)

default 경로 (`<repo>/models/<service>/`) 를 그대로 쓰면 이 섹션 스킵 가능. 외부 디스크에 두려면:

```bash
# ~/.bashrc
export MODELS_ROOT=${HOME}/perspective_models    # 또는 /data/...

export SAM2_WEIGHTS=${MODELS_ROOT}/sam2
export FOUNDATIONPOSE_WEIGHTS=${MODELS_ROOT}/foundationpose
export HAPPYPOSE_WEIGHTS=${MODELS_ROOT}/happypose
export MEGAPOSE_MESHES=${MODELS_ROOT}/megapose/meshes
export HAPPYPOSE_DATA_DIR=${HAPPYPOSE_WEIGHTS}    # happypose 공식 CLI가 참조
export BUNDLESDF_WEIGHTS=${MODELS_ROOT}/bundlesdf
```

미설정 시 docker-compose는 `<repo>/models/<service>/` fallback.

---

## 4. 다운로드 가이드

쉬운 것부터 어려운 것 순서. 모든 명령은 `<repo>/models/...` 또는 §3에서 export한 경로를 전제.

### 4.1 SAM2 — 단일 파일

Meta FAIR 공식 배포. 인증/쿼터 없음.

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp/models/sam2
wget https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_large.pt
ls -lh sam2_hiera_large.pt        # ~900MB 확인
```

### 4.2 FoundationPose — refiner + scorer 가중치

NVlabs 공식 Google Drive에서만 제공. 두 디렉토리 (`2023-10-28-18-33-37` = refiner, `2024-01-11-20-02-45` = scorer) 안에 각각 `model_best.pth`. 받아서 `weights/` 에 **리네임하여** 배치.

**방법 A — 브라우저로 수동 (가장 안정적)**

1. [NVlabs FoundationPose Download weights](https://drive.google.com/drive/folders/1DFezOAD0oD1BblsXVxqDsl8fj0ozB1WU) 접속
2. `2023-10-28-18-33-37/model_best.pth` 다운로드
3. `2024-01-11-20-02-45/model_best.pth` 다운로드
4. 리네임 + 배치:
   ```bash
   cd ~/ros2_ws/perspective_ws/src/perspective_grasp/models/foundationpose
   mkdir -p weights meshes
   mv ~/Downloads/2023-10-28-18-33-37_model_best.pth weights/refiner_model_best.pth
   mv ~/Downloads/2024-01-11-20-02-45_model_best.pth weights/scorer_model_best.pth
   ```

**방법 B — gdown CLI (자동화)**

```bash
pip install --user gdown            # 또는 venv 안에서

cd ~/ros2_ws/perspective_ws/src/perspective_grasp/models/foundationpose/weights

gdown --folder https://drive.google.com/drive/folders/1DFezOAD0oD1BblsXVxqDsl8fj0ozB1WU \
      -O ./tmp_fp

find ./tmp_fp -name "model_best.pth" | while read f; do
  parent=$(basename "$(dirname "$f")")
  case "$parent" in
    2023-10-28-18-33-37) mv "$f" ./refiner_model_best.pth ;;
    2024-01-11-20-02-45) mv "$f" ./scorer_model_best.pth  ;;
  esac
done
rm -rf ./tmp_fp
```

⚠️ Google Drive는 하루 다운로드 쿼터가 있어 대용량 파일은 종종 실패. gdown 실패 시 브라우저에서 "Add shortcut to My Drive" 후 rclone 또는 수동 우회.

**메시 준비** — YOLO 클래스명과 1:1 매칭

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp/models/foundationpose/meshes

cp /path/to/ycb/002_master_chef_can/textured.obj ./master_chef_can.obj
cp /path/to/ycb/006_mustard_bottle/textured.obj  ./mustard_bottle.obj

# 매칭 검증: YOLO data.yaml의 names 리스트와 대조
ls | sed 's/\.[^.]*$//' | sort > /tmp/mesh_names.txt
```

확장자 `.obj`/`.ply`/`.stl` 모두 허용, 대소문자 무관 (소문자 통일 권장).

### 4.3 CosyPose + MegaPose — happypose CLI

호스트에 happypose 직접 설치하면 PyTorch/CUDA/Panda3d 의존성 충돌 (`memory/happypose_install_trap.md` 참조) 가능성 높음. **`cosypose:latest` 컨테이너 안에서 실행을 권장**한다. 컨테이너가 아직 빌드 안 됐으면 [build.md](./build.md) §Rebuilding Docker images 참조.

**컨테이너 interactive 진입**

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp
docker compose -f docker/docker-compose.yml run --rm \
  -e HAPPYPOSE_DATA_DIR=/ws/models/happypose \
  cosypose bash
```

compose가 이미 `${HAPPYPOSE_WEIGHTS:-../models/happypose}` → `/ws/models/happypose`를 마운트하므로 `-v` 추가 불필요.

**컨테이너 내부에서 다운로드**

```bash
# CosyPose YCB-V (coarse + refiner 필수)
python -m happypose.toolbox.utils.download --cosypose_models \
    detector-bop-ycbv-pbr--970850 \
    coarse-bop-ycbv-pbr--724183 \
    refiner-bop-ycbv-pbr--604090

# MegaPose 전체 모델 (RGB + RGBD variant 모두)
python -m happypose.toolbox.utils.download --megapose_models

# (선택) YCB-V BOP 데이터셋 — 평가/렌더링 검증용
python -m happypose.toolbox.utils.download --bop_dataset ycbv
python -m happypose.toolbox.utils.download --bop_extra_files ycbv
```

| 다운로드 플래그 | 위치 | 크기 | 필수 여부 |
|---|---|---|---|
| `--cosypose_models detector-bop-ycbv-pbr--970850` | `experiments/detector-bop-ycbv-pbr/` | ~500MB | YOLO 대신 쓸 때만 |
| `--cosypose_models coarse-bop-ycbv-pbr--724183` | `experiments/coarse-bop-ycbv-pbr/` | ~300MB | ✓ 필수 |
| `--cosypose_models refiner-bop-ycbv-pbr--604090` | `experiments/refiner-bop-ycbv-pbr/` | ~300MB | ✓ 필수 |
| `--megapose_models` | `experiments/megapose-models/` (4개 run_id) | ~2GB | MegaPose 쓸 때 ✓ |
| `--bop_dataset ycbv` | `bop_datasets/ycbv/` | ~15GB | 평가 시에만 |
| `--bop_extra_files ycbv` | `bop_datasets/ycbv/` | ~100MB | 평가 시에만 |

**MegaPose variant** 실사용 가능 정리:
- `coarse-rgb-906902141/` → RGB coarse ✓
- `refiner-rgb-653307694/` → RGB refiner ✓
- `coarse-rgbd-.../` → RGBD coarse ✗ 백엔드 reject
- `refiner-rgbd-.../` → RGBD refiner ✗ 백엔드 reject

디스크 절약을 위해 RGBD 삭제 가능하나, 향후 depth 지원 작업 시 재다운로드 필요.

**메시 준비** — flat 디렉토리, `.obj`/`.ply`만

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp/models/megapose/meshes

cp /path/to/ycb/006_mustard_bottle/textured.obj ./mustard_bottle.obj
cp /path/to/ycb/002_master_chef_can/textured.obj ./master_chef_can.obj

# FoundationPose 메시와 공유하려면 symlink (포맷 호환 시)
ln -sf ../../foundationpose/meshes/*.obj .
ln -sf ../../foundationpose/meshes/*.ply .
```

**메시 단위 검증** — 설정 전 실제 단위 확인

```bash
python3 -c "
import trimesh
m = trimesh.load('$(pwd)/mustard_bottle.obj')
print('extents:', m.extents)
# ~0.1 수준이면 meters, ~100 수준이면 millimeters
"
```

결과에 따라 `cosypose_params.yaml` / `megapose_params.yaml`의 `mesh_units`를 `"m"` 또는 `"mm"`으로 조정.

### 4.4 BundleSDF — 가장 가벼움 (mesh-free)

BundleSDF는 mesh-free zero-shot tracker라 pretrained weights 필수 아님. **쓰기 가능한 `out/` 디렉토리만 있으면 동작**.

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp/models/bundlesdf
mkdir -p out
chmod u+w out
```

**(선택) LoFTR 가중치** — feature matching용, 컨테이너 빌드에 내장 안 됐을 경우

```bash
# 컨테이너 안에서 먼저 존재 확인
docker compose -f docker/docker-compose.yml run --rm bundlesdf \
  ls /opt/BundleSDF/BundleTrack/LoFTR/weights/
```

`outdoor_ds.ckpt`가 없다면 [LoFTR 공식 Google Drive](https://drive.google.com/drive/folders/1DOcOPZb3-5cWxLqn256AhwUVjBPifhuf)에서 받아 `models/bundlesdf/weights/loftr/outdoor_ds.ckpt`에 배치 + `docker-compose.yml`의 volume mount 조정.

**(선택) XMem 가중치** — 현 파이프라인은 SAM2가 마스크 공급하므로 불필요.

---

## 5. 다운로드 검증

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp

# SAM2
ls -lh models/sam2/sam2_hiera_large.pt

# FoundationPose
find models/foundationpose -type f | sort
# 기대:
#   models/foundationpose/weights/refiner_model_best.pth
#   models/foundationpose/weights/scorer_model_best.pth
#   models/foundationpose/meshes/<class>.(obj|ply|stl)  (최소 1개)

# happypose
ls models/happypose/experiments
# 기대: coarse-bop-ycbv-pbr/, refiner-bop-ycbv-pbr/, megapose-models/

# MegaPose meshes
ls models/megapose/meshes
# 기대: <class>.obj 또는 <class>.ply (최소 1개)

# BundleSDF
ls -la models/bundlesdf/out
# 기대: 비어있어도 됨, 단 사용자가 쓰기 가능
```

---

## 6. Gotchas 요약

1. **FoundationPose 가중치는 Google Drive 수동 다운로드가 유일한 공식 경로**. 쿼터에 걸리면 24h 대기.
2. **happypose CLI는 컨테이너 내부에서 실행** — 호스트 설치 시 의존성 충돌 위험.
3. **메시 파일명 ↔ YOLO 클래스명 매칭**이 가장 자주 실수하는 포인트. YOLO `data.yaml`의 `names` 리스트를 그대로 (소문자, 공백은 underscore) 사용.
4. **mesh 단위 확인 누락 시 pose가 1000배 어긋남**. 첫 검증 전 trimesh로 extents 출력 확인.
5. **SAM2 체크포인트 파일명 고정** — `sam2_hiera_large.pt`. 다른 버전 원하면 params yaml 직접 수정.
6. **MegaPose RGBD variant는 현재 백엔드가 reject** — RGB variant만 사용 가능.
