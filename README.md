# motion-sim

Unitree G1 29-DoF의 키프레임 모션을 편집하고, Isaac Sim에서 GR00T Decoupled WBC 정책으로 sim2sim 검증하는 편집툴이다. 이전 프로젝트 Motion Creator(MuJoCo + React)의 경험을 바탕으로 Isaac Sim + viser로 다시 만든다. 이전 설계는 [`docs/motioncreator-architecture.md`](docs/motioncreator-architecture.md)에 있다.

> 상태: v0.1 기구학 편집기. 계획과 로드맵은 [`docs/plan.md`](docs/plan.md)에 있다. Isaac Sim·Decoupled WBC 연동은 v0.3에서 한다.

## 편집기 실행

```bash
conda activate motionsim
python -m motionsim.app --model g1          # 또는 --model g1-tools
# 로컬 브라우저에서 http://<서버>:8080
python -m pytest                            # 회귀 테스트
```

- 파란 구를 클릭하면 선택되고 기즈모가 붙는다. 이동·회전하면 전신 IK로 자세를 맞춘다. 빨간 구는 위치 고정된 부위다.
- `선택 부위` 패널에서 위치·각도 고정과 IK 모드를 바꾸고, `관절` 패널에서 개별 관절각을 조정한다.
- `키프레임` 패널에서 현재 자세를 추가·갱신한다. 하단 타임라인의 마커를 끌면 구간 시간이 바뀌고, 프레임을 누르면 보간 자세를 보여 준다.
- 저장하면 `motions/<이름>_<id8>/`에 `project.json`, `motion.csv`, `metadata.json`이 생긴다. Motion Creator `project.json`도 불러올 수 있다(키프레임만).

## 설계 결정

| 항목 | 결정 |
|---|---|
| 모션 표현 | Motion Creator와 같이 수동 저작 키프레임을 모션으로 저장한다. |
| 편집 IK | Motion Creator의 SciPy bounded least-squares 방식을 유지하고, FK/Jacobian은 Pinocchio로 계산한다. |
| 물리·정책 | Isaac Sim 5.1 단독. 정책은 우선 GR00T Decoupled WBC(Balance/Walk ONNX)만 쓴다. |
| 프로세스 | 편집 프로세스(viser, IK)는 요청마다 응답한다. Isaac Sim은 기동에 약 40초가 걸리므로 상주 워커로 분리한다. |
| UI | viser. 타임라인 API가 있는 [kimodo-viser](https://github.com/nv-tlabs/kimodo-viser) fork를 commit으로 고정해 쓴다. |
| 로봇 모델 | 모델마다 완성 URDF를 통째로 교체한다. `g1`, `g1-tools`(오른손 주걱, 왼손 끝단 받침) |
| 모터 특성 | 질량·형상은 URDF, armature·마찰·최대 토크는 정책 학습 모델(`g1_gear_wbc.xml`)을 따른다. 자세한 값은 `integrations/robot-models.json`의 `isaac_actuators`에 있다. |

## 저장소 구성

| 경로 | 내용 |
|---|---|
| `assets/g1/` | unitree_ros `ccfc6fd`의 `g1_29dof_rev_1_0.urdf`와 메시 (BSD-3-Clause) |
| `assets/g1_scoop_endsupport/` | g1-tools 커스텀 그리퍼 URDF. **저장소에 포함하지 않는다.** 서버에 수동으로 둔다. |
| `external/decoupled-wbc/` | Decoupled WBC ONNX 정책과 참조 MJCF/yaml. git 제외. `scripts/setup-decoupled-wbc.sh`로 준비한다. |
| `integrations/` | 모델·정책 자산 manifest(SHA-256), Decoupled WBC 제어 파라미터 |
| `motionsim/` | `robot.py`(Pinocchio FK·IK), `motion.py`(프로젝트·보간·저장), `app.py`(viser 편집기) |
| `tests/` | Motion Creator 알고리즘 동등성, 프로젝트 계약 테스트 |
| `scripts/` | env 설치, 자산 준비, 검증 |
| `requirements.lock.txt` | `motionsim` env의 정확한 패키지 버전 |

## 실행 환경

GPU 서버(RTX PRO 6000 Blackwell 96 GB, Ubuntu 22.04)에서 개발·실행하고, 로컬 브라우저로 `http://<서버>:8080`의 viser에 접속한다.

```bash
./scripts/setup-env.sh                                   # conda env 'motionsim' 생성
conda activate motionsim
./scripts/setup-decoupled-wbc.sh /path/to/GR00T-WholeBodyControl
python scripts/verify_assets.py                          # URDF와 정책 자산의 SHA-256 검증
export OMNI_KIT_ACCEPT_EULA=YES                          # Isaac Sim 실행 시 필요
```

환경 설치 시 주의할 점:

- `isaacsim[all]`은 torch cu126 빌드를 설치한다. Blackwell(sm_120)에서 쓰려면 cu128 빌드로 강제 재설치해야 하며, 설치 스크립트가 이를 처리한다.
- isaacsim-kernel은 `websockets==12.0`을 요구하지만 viser 때문에 15.x가 설치된다. Omniverse 클라우드 검색·인증 확장만 영향을 받으며, headless 물리는 정상 동작을 확인했다.

## 정책 모델과 URDF의 차이

`g1_gear_wbc.xml`과 URDF는 관절 순서가 같지만, 다음 값이 다르다 (`integrations/robot-models.json`에 기록).

- 발목 pitch/roll, 허리 roll/pitch 최대 토크: URDF 35 Nm, 정책 모델 50 Nm. Isaac에서는 50 Nm를 쓰고, sim2real 전에 실물 한계를 확인한다.
- torso와 손목 yaw 링크 질량, 손 구성(정책 모델은 Dex3 손가락 링크 고정)이 다르다. 전체 질량은 URDF g1 33.34 kg, 정책 모델 34.39 kg이다.

## 라이선스

`assets/g1/`는 unitree_ros의 BSD-3-Clause(`assets/g1/LICENSE_unitree_ros`)를 따른다. Decoupled WBC 자산은 저장소에 포함하지 않는다. 원본 저장소 기준으로 코드는 Apache-2.0, 모델 가중치는 NVIDIA Open Model License다.
