# How to Add a New Grasp Planner

이 문서는 `grasp_pose_planner` 패키지에 새로운 grasp planning 알고리즘을 추가하는 방법을 설명합니다.

---

## 아키텍처 개요

```
grasp_pose_planner/
├── grasp_planner_node.py          # ActionServer + planner 선택 (PLANNER_REGISTRY)
└── planners/
    ├── __init__.py
    ├── base_planner.py            # ABC — 모든 planner가 상속
    └── antipodal_planner.py       # 예시 구현
```

- **`BasePlanner`** : 추상 베이스 클래스. `plan()` 메서드를 반드시 구현해야 합니다.
- **`PLANNER_REGISTRY`** : `grasp_planner_node.py`에 정의된 딕셔너리. `planner_type` 파라미터 값 → 클래스 매핑.
- **`planner_type`** : ROS 2 파라미터. YAML 또는 launch에서 설정.

---

## Step 1: Planner 클래스 작성

`planners/` 디렉터리에 새 파일을 만듭니다.

```python
# grasp_pose_planner/planners/my_planner.py

from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Callable
    from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from perception_msgs.action import PlanGrasp
from .base_planner import BasePlanner


class MyPlanner(BasePlanner):

    def __init__(self, node: Node) -> None:
        super().__init__(node)
        # 필요한 ROS 파라미터 선언
        self._my_param = (
            node.declare_parameter('my_custom_param', 42)
            .get_parameter_value().integer_value
        )

    def plan(
        self,
        goal: PlanGrasp.Goal,
        publish_feedback: Callable[[PlanGrasp.Feedback], None],
    ) -> PlanGrasp.Result:
        # 1) Feedback 전송 (optional)
        feedback = PlanGrasp.Feedback()
        feedback.current_stage = 'generation'
        feedback.num_candidates = 0
        feedback.best_score_so_far = 0.0
        publish_feedback(feedback)

        # 2) 실제 알고리즘 구현
        #    - goal.target_object_id  : 대상 오브젝트 ID
        #    - goal.grasp_strategy    : 'power' / 'precision' / 'pinch'

        # 3) 결과 반환
        result = PlanGrasp.Result()
        result.grasp_pose = PoseStamped()        # 계산된 grasp pose
        result.finger_joint_config = [0.0] * 10  # 10-DoF 관절 각도
        result.approach_waypoints = []            # 접근 경로 waypoints
        result.grasp_quality_score = 0.95
        result.success = True
        result.message = 'MyPlanner succeeded'
        return result
```

### `plan()` 규약

| 항목 | 설명 |
|------|------|
| `goal` | `PlanGrasp.Goal` — `target_object_id` (int32), `grasp_strategy` (string) |
| `publish_feedback` | `PlanGrasp.Feedback`를 클라이언트에 전송하는 콜백 |
| 반환값 | `PlanGrasp.Result` — `success=True/False` 필수 |
| `current_stage` | `'generation'` → `'feasibility'` → `'selection'` 순서 권장 |

---

## Step 2: Registry에 등록

`grasp_planner_node.py`의 `PLANNER_REGISTRY`에 항목을 추가합니다.

```python
PLANNER_REGISTRY: dict[str, str] = {
    'antipodal': 'grasp_pose_planner.planners.antipodal_planner:AntipodalPlanner',
    'my_planner': 'grasp_pose_planner.planners.my_planner:MyPlanner',  # 추가
}
```

key(`'my_planner'`)가 YAML의 `planner_type` 값과 일치해야 합니다.

---

## Step 3: YAML 파라미터 설정

`config/grasp_planner_params.yaml`을 수정합니다.

```yaml
grasp_planner:
  ros__parameters:
    planner_type: "my_planner"    # ← 여기만 변경하면 planner 교체

    # MyPlanner 전용 파라미터
    my_custom_param: 42
```

---

## Step 4: 빌드 & 실행

```bash
# 빌드
cd ~/ros2_ws/perspective_ws
colcon build --packages-select grasp_pose_planner
source install/setup.bash

# 실행
ros2 launch grasp_pose_planner grasp_planner.launch.py
```

빌드 후 `planner_type` 파라미터만 바꾸면 리빌드 없이 planner를 교체할 수 있습니다.

---

## Step 5 (optional): Launch에서 override

YAML 수정 없이 launch 시 파라미터를 덮어쓸 수 있습니다.

```bash
ros2 launch grasp_pose_planner grasp_planner.launch.py \
    --ros-args -p planner_type:=my_planner
```

또는 launch 파일에서:

```python
Node(
    package='grasp_pose_planner',
    executable='grasp_planner_node',
    name='grasp_planner',
    parameters=[config, {'planner_type': 'my_planner'}],
    output='screen',
)
```

---

## Action 테스트

```bash
# Terminal 1: 노드 실행
ros2 launch grasp_pose_planner grasp_planner.launch.py

# Terminal 2: goal 전송
ros2 action send_goal /plan_grasp perception_msgs/action/PlanGrasp \
    "{target_object_id: 1, grasp_strategy: 'power'}" --feedback
```

---

## 요약

| 단계 | 파일 | 작업 |
|------|------|------|
| 1 | `planners/my_planner.py` | `BasePlanner` 상속, `plan()` 구현 |
| 2 | `grasp_planner_node.py` | `PLANNER_REGISTRY`에 등록 |
| 3 | `config/grasp_planner_params.yaml` | `planner_type` 변경 |
| 4 | — | `colcon build` → 실행 |
