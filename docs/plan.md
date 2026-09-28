# motion-sim 기획

> 기준일: 2026-09-28 · 대상: 이 저장소를 개발하는 사람과 AI 에이전트
> 이전 프로젝트 설계는 [`motioncreator-architecture.md`](motioncreator-architecture.md)를 참고한다. 이 문서는 그중 무엇을 가져오고 무엇을 바꾸는지를 정한다.

## 1. 목표

Unitree G1 29-DoF의 키프레임 모션을 브라우저에서 편집한다. 물체를 배치하고 상자를 양손으로 파지하는 장면까지 저작한 뒤, Isaac Sim에서 GR00T Decoupled WBC로 추종시켜 sim2sim으로 검증한다. 이 프로젝트의 목적은 **편집툴**이다. sim2real은 sim2sim 이후 별도로 진행한다.

## 2. Motion Creator와의 관계

| 영역 | Motion Creator | motion-sim | 이유 |
|---|---|---|---|
| 모션 표현 | 수동 키프레임, `duration` = 직전 키프레임에서 오는 시간 | **그대로** | 검증된 의미. 파라미터 segment 방식은 채택하지 않음 |
| 편집 IK | SciPy bounded least-squares, MuJoCo FK/Jacobian | **알고리즘 유지, FK/Jacobian은 Pinocchio** | URDF 하나를 Isaac과 공유. MC 결과와 회귀 비교 |
| 보간 | quintic easing, root SLERP, 공통 pin은 IK로 재투영 | **그대로** | |
| 로봇 모델 | MJCF(g1), URDF→MJCF 변환(g1-tools) | **완성 URDF 통째 교체** | Pinocchio와 Isaac이 같은 파일을 읽음 |
| UI | React + Three.js, FastAPI | **viser(kimodo-viser fork), Python 단일 프로세스** | 타임라인 API 제공, 프런트 빌드 불필요 |
| 물리 | MuJoCo PD / SONIC / Decoupled WBC | **Isaac Sim 상주 워커 + Decoupled WBC** | 물리 기준을 Isaac 하나로 둠 |
| 저장 | `motioncreator.g1.v1`, NPZ/CSV/JSON 번들 | `motionsim.g1.v1`, CSV/JSON 번들 (NPZ는 이후) | MC 프로젝트 가져오기 지원 |

## 3. 아키텍처

```text
로컬 브라우저 ──(viser websocket :8080)── 편집 프로세스 (GPU 서버, env motionsim)
                                           ├─ viser UI: 3D 뷰, 핸들·기즈모, 패널, 타임라인
                                           ├─ Robot: Pinocchio FK + bounded least-squares IK
                                           ├─ Motion: 프로젝트 검증, 키프레임 보간, 저장
                                           └─ (v0.2) Scene: 물체 배치
                                                  │ 프로젝트 스냅샷 → replay 결과
                                                  ▼
                                           Isaac 워커 (v0.3, 같은 env, headless 상주)
                                           ├─ URDF → USD, 모터 특성 적용
                                           ├─ Decoupled WBC: 하체 15 DoF 정책 + 팔 14 DoF PD
                                           └─ 추종 오차·넘어짐·접촉 기록
```

- 편집 FK/IK는 요청마다 바로 응답한다. Isaac은 기동에 약 40초가 걸리므로 계속 띄워 두고, 작업마다 장면만 초기화한다.
- 편집 프로세스와 Isaac 워커는 프로젝트 스냅샷(JSON)과 결과 배열로만 통신한다.

## 4. 데이터 계약

### 4.1 qpos

- `[root xyz(3), root quaternion wxyz(4), 관절 29개]` = 36개 값. Motion Creator, Kimodo, CSV와 같다.
- 관절 순서는 URDF revolute 선언 순서이고, 정책 MJCF `g1_gear_wbc.xml`의 hinge 순서와 같다.
- Pinocchio 내부 표현(quaternion xyzw, 트리 순회 순서)과 Isaac DOF 순서는 **경계에서만 변환**한다. 변환은 관절 이름 기준이다.
- viser/Three.js의 quaternion은 wxyz다. MC의 Three.js(xyzw)와 다르므로, 경계에서 순서를 명시한다.

### 4.2 프로젝트 (`motionsim.g1.v1`)

```jsonc
{
  "format": "motionsim.g1.v1",
  "project_id": "32 hex", "created_at": "ISO 8601", "name": "",
  "model_id": "g1 | g1-tools", "model_sha256": "URDF SHA-256", "joint_names": ["29개"],
  "coordinate_system": "right-handed, +X forward, +Y left, +Z up",
  "units": {"position": "m", "angle": "rad", "time": "s"},
  "keyframes": [{"name": "", "duration": 2.0, "qpos": [], "pins": [], "angle_pins": []}],
  "current_qpos": [], "pins": [], "angle_pins": []
}
```

- 검증 한계는 MC와 같다. 키프레임 1–100개, duration 0.1–60 s, 총 길이 600 s 이하.
- 호환 규칙: `joint_names`가 같아야 한다. `model_sha256`은 현재 모델이거나, g1-tools에서 g1 프로젝트를 여는 경우만 허용한다(관절이 같고 끝단만 다름).
- MC 프로젝트(`motioncreator.g1.v1`)는 관절 순서가 같을 때 키프레임만 가져온다. 장면·파지·클립은 해당 기능이 생길 때까지 무시하고, 무시한 내용을 경고로 알린다.

### 4.3 저장 번들

`motions/<이름>_<id8>/`에 `project.json`, `motion.csv`(헤더 없는 qpos 36열), `metadata.json`을 원자적으로 저장한다. `motions/`는 사용자 데이터이므로 git에서 제외한다.

## 5. 로드맵

순서는 v0.1 → v0.3 → v0.2 → v0.4 → v0.5로 바꿨다(2026-09-28). 가장 큰 리스크인 Isaac·WBC를 먼저 확인하기 위해서다.

| 버전 | 범위 | 완료 기준 | 상태 |
|---|---|---|---|
| **v0.1 기구학 편집기** | Pinocchio Robot(FK/IK), 프로젝트·보간·저장, viser 편집기(핸들 드래그, 회전, 관절 슬라이더, pin, 키프레임, 타임라인, 재생, 저장·열기), MC 프로젝트 가져오기 | MC fixture 회귀 테스트 통과, 브라우저에서 저작·재생·저장 가능 | 완료. UI 개편, 실행 취소 포함 |
| **v0.3 Isaac + Decoupled WBC** | 상주 워커, URDF 가져오기, 모터 특성 적용, 정책 관측 구성, replay | 저작 모션을 Isaac에서 WBC로 추종하고 결과를 편집기에서 재생 | 완료. 7절 |
| v0.2 장면 | 기본 도형과 USD/GLB 물체 배치, 그룹, 지면 고정 | 물체가 저장·복원되고 Isaac 장면 빌더 입력으로 쓸 수 있음 | 다음 |
| v0.4 양손 파지 | 상자 접촉면 지정, 파지 자세 맞춤, Isaac 접촉력 검증 | 상자를 들고 옮기는 모션의 접촉력·미끄러짐 기록 | |
| v0.5 내보내기 | Kimodo NPZ 등 외부 형식 | MC와 같은 NPZ 계약 | |

## 6. v0.1 상세

### 6.1 모듈

| 파일 | 책임 |
|---|---|
| `motionsim/robot.py` | 모델 로드, 핸들 정의, qpos 변환, FK 상태, IK(`Robot.solve`) |
| `motionsim/motion.py` | 프로젝트 생성·검증, MC 가져오기, 보간 컴파일, 번들 저장·열기 |
| `motionsim/app.py` | viser 편집기 UI와 상태 |
| `tests/` | MC fixture 회귀, 프로젝트 계약, 보간 테스트 |

### 6.2 편집기 동작

- 핸들 구(sphere)를 클릭하면 선택되고 기즈모가 붙는다. 이동하면 위치 목표, 회전하면 방향 목표(회전 가능한 핸들만)로 IK를 푼다.
- 드래그 중 IK는 직렬화한다. 최신 목표 하나만 남기고 이전 요청은 버린다(MC의 `pending` 방식).
- 드래그 시작 자세가 IK의 anchor다. elastic/free 모드와 resistance를 패널에서 바꾼다.
- 선택한 핸들의 위치 고정, 각도 고정을 토글한다. 기본은 양발 위치 고정이다.
- 관절 슬라이더는 관절각 목표로 IK를 풀어, pin이 유지되게 한다.
- 키프레임: 추가(선택 키프레임 뒤), 현재 자세로 갱신, 삭제, 이름·duration 편집, 선택하면 자세 불러오기.
- 타임라인: 키프레임 마커를 표시한다. 마커를 드래그하면 duration이 바뀌고, 프레임을 클릭하면 보간 자세를 보여 준다.
- 재생: 30 FPS 컴파일 결과를 재생한다. 재생 중에는 편집을 잠근다.

### 6.3 알려진 한계 (v0.1)

- 기구학 결과일 뿐이다. 균형·추종은 v0.3 Isaac 검증에서 본다.
- 다중 선택과 좌우 미러는 아직 없다. 실행 취소·다시 실행은 추가했다(100단계).
- 단일 사용자 기준이다. 여러 브라우저가 붙으면 같은 편집 상태를 공유한다.
- 표시용 메시는 약 27%로 줄여 보낸다(`--full-meshes`로 원본). IK와 Isaac은 원본 URDF를 쓴다.

## 7. v0.3 Isaac + Decoupled WBC

### 7.1 구조

| 파일 | 책임 |
|---|---|
| `motionsim/wbc.py` | 물리 엔진과 무관한 제어기. 하체 15 DoF는 Balance/Walk ONNX(50 Hz) → PD 토크(200 Hz), 팔 14 DoF는 저작 궤적 PD + 편향 토크, 루트 궤적 → 속도·yaw rate·높이 명령, 허리 관절 → torso RPY 명령, 낙상 판정, 25 Hz 기록 |
| `motionsim/sim_isaac.py` | Isaac 백엔드. URDF 가져오기, 관절 속성 작성, 상태 읽기(루트 각속도는 루트 좌표계), 팔 편향 토크는 같은 URDF로 Pinocchio RNEA |
| `motionsim/sim_mujoco.py` | 검증 전용 MuJoCo 백엔드(정책 모델 `g1_gear_wbc.xml`) |
| `motionsim/isaac_worker.py`, `isaac_client.py` | 상주 워커(Isaac 한 번 기동, 작업마다 reset)와 편집기 쪽 연결. 워커는 편집기가 끝나면 스스로 종료 |
| 편집기 `검증` 탭 | 워커 시작, 현재 모션 검증, 진행률, 결과 카드(완주/낙상, 추종 오차), 물리 결과 재생 |

제어기 호출 규약: 물리 스텝마다 `compute_torques(state, arm_bias)` → 스텝 → `after_step(state)`. `state`는 motion-sim 규약(qpos wxyz, URDF 관절 순서, 루트 각속도는 루트 좌표계)이다. Isaac의 DOF 순서는 관절 이름으로 매핑한다.

### 7.2 검증 결과 (2026-09-28)

1. **제어기 이식**: Motion Creator `DecoupledSimulation`(MuJoCo)을 직접 실행한 기준값(서기 → 손 뻗기 → 0.3 m 걷기 + yaw 20°, 8 s, 1600 스텝)과 같은 모델에서 비교했다. 궤적 차이는 1e-5 미만이다(`tests/test_wbc.py`).
2. **Isaac vs MuJoCo** (같은 제어기, 같은 모션, `scripts/sim2sim_check.py`, `pytest -m isaac`):

| | MuJoCo (정책 모델) | Isaac g1 | Isaac g1-tools |
|---|---|---|---|
| 8 s 완주 | 예 | 예 | 예 |
| 최종 루트 xy (목표 0.3, 0) | (0.312, −0.004) | (0.290, 0.008) | (0.292, −0.014) |
| 최종 yaw (목표 20°) | 18.8° | 18.7° | 18.8° |
| 팔 추종 RMSE | 0.44° | 0.31° | 3.18° |

Isaac–MuJoCo 궤적 차이(g1): 루트 최대 3.8 cm, 관절 RMSE 1.6°. 다리 RMSE 약 14°는 두 엔진이 같다. 다리는 정책이 걸음을 만들기 때문이며 추종 실패가 아니다.

### 7.3 발견한 문제와 결정

- **armature는 USD로 작성해야 한다.** Isaac tensor API(`set_armatures`)로 실행 중에 넣은 값은 읽을 수는 있지만 동역학에 반영되지 않았다. 손목 roll 관절이 3스텝 만에 ±37 rad/s로 발산했다. 시뮬레이션 시작 전에 `PhysxJointAPI.armature`로 작성하면 안정적이다. 같은 이유로 관절 최대 토크, 드라이브 게인(0), 속도 한계도 USD로 작성한다.
- URDF의 관절 속도 한계(37 rad/s)는 없앴다. MuJoCo에는 속도 한계가 없다.
- MuJoCo의 관절 dry friction(0.1 Nm)과 damping(0.001)은 PhysX 관절 마찰(무차원)과 모델이 달라, 명령 토크에 tanh 근사로 더한다.
- 지면 마찰은 1.0에 `max` 결합으로 두어, 로봇 기본 재질과 평균이 되지 않게 했다. 발 충돌체는 두 모델이 같다(발마다 5 mm 구 4개).
- g1-tools의 팔 RMSE가 더 큰 것은 도구 질량(주걱 0.42 kg 등) 때문이다. 팔 편향 토크는 같은 URDF로 계산해 보상한다.
- 워커 기동은 캐시가 데워진 뒤 약 10초, 6초 분량 모션 계산은 약 4초(약 1.5배속)다. 같은 입력이면 결과가 같다.
- MuJoCo 3.12를 env에 추가했다. 이식 검증과 sim2sim 비교 전용이고, 편집과 물리 판정은 Isaac만 쓴다.

### 7.4 남은 한계

- 파지 힘 피드백(MC의 `grasp_force_control`)과 물체는 아직 없다(v0.2, v0.4).
- manual 모드(키보드 명령)와 녹화 번들 저장은 아직 없다.
- 결과는 이 모델·파라미터·CPU 정책 추론에 대한 시뮬레이션이다. 실기 안전을 보장하지 않는다.

## 8. Motion Creator 모델과의 차이 (2026-09-28 확인)

IK·보간 알고리즘은 MC와 같다. 동등성은 `tests/test_motioncreator_parity.py`가 MC 코드로 만든 기준값과 비교해 확인한다(`tests/fixtures/make_motioncreator_reference.py`).

| 모델 | MC | motion-sim | 패리티 기준값 |
|---|---|---|---|
| g1 | `assets/g1/g1.xml`(구형): `waist_roll_link` z 0.035, `torso_link` z 0.019 | unitree_ros rev_1_0 URDF: `waist_roll_joint` z 0.044, `waist_pitch_joint` z 0 | 허리 pitch 축이 10 mm 달라 MC 모델을 그대로 쓸 수 없다. MC 코드를 motion-sim URDF로 만든 MuJoCo 모델에서 실행한다. 정책 학습 모델 `g1_gear_wbc.xml`은 motion-sim과 같은 rev_1_0이다. |
| g1-tools | `tool_model.py`가 원본 그리퍼 URDF(주걱=오른손)를 좌우 반전해 **주걱 = 왼손**으로 부착 | 생성기 `--scoop-side left`로 만든 **주걱 = 왼손** URDF (실물 장착 방향, 2026-09-28 확인) | 모델이 같으므로 MC의 실제 g1-tools를 그대로 쓴다. TCP는 0.0 mm로 일치한다. |

MC g1 프로젝트를 가져오면 관절각(qpos)을 그대로 쓴다. 다리와 발은 두 모델이 같아 발 고정은 유지되고, 허리를 굽힌 자세에서는 상체 핸들 위치가 최대 약 11 mm 달라질 수 있다. MC g1-tools 프로젝트는 기구학이 같다.

## 9. 리스크

| 리스크 | 대응 |
|---|---|
| MC MJCF와 unitree URDF의 기구학 차이 | 8절에 기록. 알고리즘 회귀는 같은 URDF 기준 fixture로 비교 |
| 그리퍼 생성기의 메시 비결정성 | 같은 옵션으로 다시 만들면 URDF는 같지만 끝단 받침 본체 STL 하나가 바이트 단위로 다를 수 있다. 해시 검증은 URDF 기준 |
| 정책 학습 모델과 URDF의 질량·토크 차이 | `integrations/robot-models.json`의 `isaac_actuators` 결정을 따른다. sim2real 전에 실물 토크 한계 확인 |
| MuJoCo 관절 마찰·damping을 PhysX로 옮길 때 모델 차이 | 명령 토크에 근사로 더한다. 걷기 포함 모션에서 Isaac–MuJoCo 결과 비교(7.2절), `pytest -m isaac`로 회귀 확인 |
| 공유 GPU 서버에서 Isaac 워커가 남는 문제 | 워커가 편집기 프로세스를 감시해 편집기가 끝나면 스스로 종료. 워커는 버튼이나 `--isaac`으로만 시작 |
| websockets 버전 충돌(Isaac 12.0 vs viser 15.x) | headless 물리 검증됨. 문제 시 편집기 env와 Isaac env 분리 |
| viser fork 의존 | commit 고정, Apache-2.0 |
