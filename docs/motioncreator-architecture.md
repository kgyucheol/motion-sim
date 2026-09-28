# Motion Creator AI 프로젝트 컨텍스트

> 대상: 이 저장소를 처음 읽는 개발자와 AI 에이전트<br>
> 기준: 2026-09-28 저장소 소스<br>
> 목적: 이 문서 하나를 컨텍스트 프롬프트로 제공해 Motion Creator의 설계, 기능, 기술 스택, 데이터 계약과 소스 위치를 빠르게 파악하게 한다.

<ai_context_prompt>

당신은 `/home/kim/motioncreator` 저장소를 분석·수정하는 소프트웨어 엔지니어다. 이 문서를 프로젝트의 기준 컨텍스트로 사용하라.

<scope>

- 분석 대상은 Motion Creator 저장소 내부 코드와 이 코드가 직접 소유하는 데이터 계약이다.
- `/home/kim/GR00T-WholeBodyControl`, `/home/kim/ProtoMotions`, `/home/kim/ardy` 등 이웃 저장소의 내부 설계는 범위 밖이다.
- SONIC, GR00T Decoupled WBC, ProtoMotions, ARDY는 Motion Creator가 호출하거나 변환하는 **외부 연동 경계**로만 이해하라.
- `motions/`, `tasks/`, `presets/`, 미추적 `assets/`에는 사용자 생성 데이터가 있을 수 있으므로 요청 없이 수정·삭제·커밋하지 마라.

</scope>

<primary_goal>

Motion Creator는 Unitree G1 29-DoF 로봇의 자세와 키프레임 모션을 시각적으로 저작하고, 장면·파지 조건을 함께 저장하며, 기구학 및 선택적 물리/정책 실행으로 결과를 검토하는 로컬 웹 애플리케이션이다.

</primary_goal>

<reasoning_rules>

1. 기능을 판단할 때 README의 설명만 믿지 말고 아래에 연결된 `파일 — 함수/클래스`를 확인하라.
2. 메인 편집기, Decoupled WBC, CPU 상자 태스크 PoC의 실행 경계를 섞지 마라.
3. `qpos` quaternion의 `wxyz`와 Three.js/장면 quaternion의 `xyzw`를 항상 구분하라.
4. 기구학 결과, PD 물리 결과, 학습 정책 결과, 실제 접촉 측정의 의미를 구분하라.
5. 데이터 필드를 바꾸기 전 TypeScript 상태, Python 검증, 저장 형식, 물리 builder와 테스트의 연결점을 모두 추적하라.
6. 변경 후 영향 범위에 맞는 Python 테스트, TypeScript build, 장면 테스트를 실행하라.
7. 기존 사용자 데이터와 관련 없는 변경을 커밋에 포함하지 마라.

</reasoning_rules>

<source_reference_format>

이 문서의 소스 근거는 `상대/경로 — 함수명`, `상대/경로 — 클래스명`, 또는 모듈 상수명으로 표시한다. 정확한 구현이 필요하면 해당 심볼과 직접 호출자를 먼저 읽어라.

</source_reference_format>

</ai_context_prompt>

## 1. 한눈에 보는 프로젝트

Motion Creator는 Unitree G1 29-DoF 휴머노이드용 로컬 모션 저작·검증 도구다. 사용자는 브라우저의 Three.js 장면에서 로봇 부위를 선택해 위치 또는 회전을 조작하고, Python 서버는 같은 MuJoCo 모델을 이용해 전신 IK와 FK를 계산한다. 완성한 키프레임은 시간축 모션으로 보간하고, Kimodo 호환 NPZ·CSV·편집 가능한 JSON으로 저장한다. 선택적으로 MuJoCo 물리와 GEAR-SONIC 정책으로 추종 가능성을 검토하거나, GR00T Decoupled WBC로 상체 모션과 하체 보행 정책을 결합할 수 있다.

핵심 특성은 다음과 같다.

- 로봇 모델: 자유 루트 7 qpos + 힌지 관절 29개 = `nq=36`, `nv=35`.
- 기준 좌표계: 오른손 좌표계, `+X` 전방, `+Y` 좌측, `+Z` 위. 위치 m, 각도 rad, 시간 s.
- 저작 방식: 부위 핸들, 개별 관절, 다중 선택, 좌우 미러, 위치·방향 고정 조건을 조합한 전신 IK.
- 시간축: 의미론적 키프레임 또는 가져온 단일 모션 클립. 키프레임 사이에는 quintic easing과 quaternion 최단 경로 보간을 사용한다.
- 장면: 기본 도형과 가져온 GLB를 배치·그룹화하며, 저장·물리 모델에도 전달한다.
- 파지: 상자 양면의 접촉점을 지정하고 손목 후보와 양팔 IK를 계산한다. 물체를 손에 weld하지 않고 물리 접촉력으로 검증한다.
- 실행: 로컬 FastAPI 서버가 빌드된 React/Vite 앱과 API를 같은 포트에서 제공한다.
- 선택적 통합: Kimodo G1Skeleton34, ProtoMotions 3, GEAR-SONIC, GR00T Decoupled WBC, 실험적 ARDY 상자 태스크.

대표 소스 레퍼런스:

- `motioncreator/server.py` — `main`, `initialize`, `solve_group`, `save`
- `motioncreator/robot.py` — `Robot`, `Robot.solve`, `Robot.state`
- `motioncreator/motion.py` — `validate_project`, `compile_motion`, `save_bundle`
- `frontend/app/page.tsx` — `Editor`
- `frontend/lib/robot-scene.ts` — `RobotScene`

## 2. 무엇이 주 제품이고 무엇이 보조 기능인가

처음 코드를 읽을 때 아래 경계를 먼저 구분해야 한다.

| 구분 | 진입점 | 현재 역할 |
|---|---|---|
| 메인 모션 편집기 | `/` | 자세 편집, 키프레임, 장면, 파지, 저장, 기구학·물리 미리보기의 주 제품 |
| Decoupled WBC | `/decoupled-wbc` | 저장한 모션의 양팔을 직접 추종하고 허리·다리는 GR00T 정책이 제어하는 별도 워크벤치 |
| CPU 상자 태스크 PoC | `/api/tasks/*` | ARDY 참조 생성과 SONIC/MuJoCo 팔레타이징 검증을 위한 서버 API·레거시 실험 흐름 |
| `TaskWorkbench` | 직접 라우팅되지 않음 | CPU 상자 태스크용 UI 코드가 남아 있으나 현재 `main.tsx`에서 렌더링하지 않음 |
| ProtoMotions 변환 | 저장 옵션·CLI | 저작 결과를 검증된 ProtoMotions `.motion`/`.pt`로 파생 변환 |

라우팅 근거:

- `frontend/app/main.tsx` — 최상위 경로 선택
- `frontend/app/page.tsx` — `Editor`
- `frontend/app/decoupled-wbc-page.tsx` — `DecoupledWbcPage`
- `frontend/app/task-workbench.tsx` — `TaskWorkbench`
- `motioncreator/server.py` — `app.include_router`

## 3. 실행 아키텍처

```text
Browser
  React Editor / Decoupled WBC page
    ├─ Three.js RobotScene: 화면, 선택, 기즈모, 레이캐스팅
    ├─ 편집 상태: project, pose, pins, objects, preview
    └─ HTTP / WebSocket
                │
                ▼
FastAPI server (motioncreator.server)
    ├─ Robot: MuJoCo FK + bounded whole-body IK
    ├─ Motion: 프로젝트 검증, 보간, 가져오기, 원자적 저장
    ├─ Scene/Grasp: 외부 모델, 충돌 프록시, 양손 파지
    ├─ Policy preview worker: PD 또는 GEAR-SONIC + MuJoCo
    ├─ Decoupled WBC simulation thread
    └─ Task worker subprocess: ARDY/SONIC 실험 파이프라인
                │
                ▼
Local files
  assets/  motions/  presets/  tasks/  integrations/  external/
```

서버는 시작할 때 하나의 전역 `Robot`과 `GroupStore`를 만든다. 로봇 모델을 바꾸면 실행 중인 물리/WBC 작업이 없는지 확인한 뒤 전역 모델을 교체하고 GLB를 다시 내보낸다. 일반 편집 FK/IK는 요청-응답 방식이다. 시간이 긴 정책 물리 계산은 별도 프로세스와 임시 폴더를 사용하고, Decoupled WBC는 세션별 백그라운드 스레드에서 계속 스텝한다.

소스 레퍼런스:

- `motioncreator/server.py` — `robot`, `groups`, `select_model`, `main`
- `motioncreator/policy_preview.py` — `PreviewJobs`, `run_persistent_worker`
- `motioncreator/decoupled_wbc.py` — `DecoupledSimulation`, `DecoupledSessions`
- `motioncreator/task_jobs.py` — `start_run`

## 4. 기술 스택과 실행 방법

### 4.1 런타임별 기술 스택

| 계층 | 기술 | Motion Creator에서의 역할 | 버전 기준 |
|---|---|---|---|
| 언어 | Python | API, IK/FK, 파일 변환, 물리·정책 worker | 3.11 |
| API 서버 | FastAPI + Starlette + Uvicorn | REST/WebSocket, 정적 SPA 제공, Pydantic 입력 검증 | lock 기준 FastAPI 0.141.1, Uvicorn 0.52.4 |
| 데이터 검증 | Pydantic v2 | API payload와 CPU task spec의 엄격한 schema | 2.13.5 |
| 로봇/물리 | MuJoCo Python | MJCF 로딩, FK/Jacobian, free-base dynamics, contact/force | 3.12.0 |
| 수치 계산 | NumPy + SciPy | 배열, bounded least-squares IK, Rotation/SLERP, 보간 | 2.4.6 / 1.17.1 |
| 3D 자산 처리 | trimesh | GLB 검증·분리, bounds 계산, 렌더 자산 준비 | 5.1.0 |
| 테스트 | pytest | Python 단위·통합·API·물리 회귀 테스트 | 9.1.1 |
| UI | React + React DOM | 편집기와 WBC 워크벤치 상태·컴포넌트 | 19.2.6 |
| 프런트 언어 | TypeScript | pose/scene/project 타입과 UI 로직 | 5.9.3 |
| 빌드 | Vite | 개발 서버와 production SPA bundle | 8.0.13 |
| 3D 렌더링 | Three.js | GLB 렌더링, 카메라, raycast, transform gizmo | 0.185.x |
| 스타일/UI | 직접 작성한 CSS + lucide-react | 편집기/WBC 레이아웃, 상태 표현, 아이콘 | `frontend/app/globals.css`, `frontend/package.json` |
| 선택적 추론 | ONNX Runtime CPU | GEAR-SONIC과 Decoupled WBC 정책 실행 | `.conda-policy` lock 기준 |
| 선택적 텐서/변환 | CPU PyTorch | ARDY 생성과 ProtoMotions `RobotState` 변환 | 별도 환경 lock 기준 |
| 자산 제작 도구 | Blender Python API | GLB export, 스캔 정리, 포장 장면 생성 | 런타임 앱과 분리 |

UI는 React local state와 ref를 중심으로 구성하며 Redux, Zustand, React Query 같은 별도 전역 상태/서버 캐시 라이브러리를 사용하지 않는다. 현재 핵심 화면 스타일은 `frontend/app/globals.css`의 직접 작성한 CSS가 담당한다. Tailwind와 shadcn 계열 패키지 및 `frontend/components/ui/` scaffold도 설치되어 있지만 메인 `Editor`와 `DecoupledWbcPage`의 주 UI 경로는 대부분 native element와 전용 CSS를 사용한다. 로봇 도메인 상태와 동작은 `frontend/app/`, `frontend/lib/`가 소유한다.

버전 근거:

- `requirements.lock.txt` — 기본 Python 환경의 정확한 패키지 버전
- `frontend/package.json` — Node engine, 프런트 의존성과 script
- `integrations/task-cpu-requirements.lock.txt` — 정책/ARDY CPU 환경
- `integrations/protomotions-requirements.lock.txt` — ProtoMotions 변환 환경

### 4.2 환경 분리

| 환경 | 소유 기능 | 준비 명령 |
|---|---|---|
| `.conda` | 편집기 API, MuJoCo IK/FK, 기본 테스트 | `./setup.sh` |
| `.conda-policy` | ONNX 정책, ARDY task worker, Decoupled WBC | `./scripts/setup-task-cpu.sh` |
| `.conda-protomotions` | ProtoMotions 파생 출력 검증·생성 | `./scripts/setup-protomotions.sh` |
| `frontend/node_modules` | React/Three.js 빌드와 TS 테스트 | `npm --prefix frontend ci` |

외부 모델과 대형 자산은 Git에 직접 넣지 않고 `external/` 아래 준비한 뒤 `integrations/*.json`의 크기·SHA-256·upstream revision으로 검증한다. Decoupled WBC 자산 복사는 `scripts/setup-decoupled-wbc.sh`가 담당한다.

### 4.3 실행

```bash
./setup.sh
./run.sh
# http://127.0.0.1:8765
```

`run.sh`는 `.conda-policy`가 있으면 이를 우선 사용하고, 없으면 `.conda`를 사용한다. 빌드 산출물 `frontend/dist/index.html`이 없으면 프런트엔드를 먼저 빌드한다. `--model g1-tools`를 주면 도구 장착 로봇으로 시작하며, 그 뒤 인자는 서버의 `--port` 등에 전달된다.

개발 중 프런트엔드만 별도 실행할 때는 `npm --prefix frontend run dev`를 사용할 수 있지만, 일반 사용은 Python 서버 하나가 정적 파일과 API를 함께 제공한다.

소스 레퍼런스:

- `run.sh` — 실행 환경 선택과 서버 시작
- `setup.sh` — Python/npm 설치와 프로덕션 빌드
- `frontend/package.json` — `dev`, `build`, `test:scene`
- `motioncreator/server.py` — `main`

## 5. 저장소 구조

| 경로 | 책임 | 먼저 볼 소스 |
|---|---|---|
| `motioncreator/` | Python 도메인·API·시뮬레이션 | `server.py`, `robot.py`, `motion.py` |
| `frontend/app/` | 화면과 사용자 흐름 | `page.tsx`, `decoupled-wbc-page.tsx` |
| `frontend/lib/` | Three.js 장면·도메인 유틸리티 | `robot-scene.ts`, `scene-objects.ts` |
| `assets/g1/` | 기본 G1 MJCF, 메시, 라이선스, 시각 GLB | `g1.xml` |
| `assets/g1_scoop_endsupport/` | 사용자 작업 중인 도구 URDF/메시 자산; 현재 Git 미추적 파일이 포함될 수 있음 | 해당 폴더 README |
| `motions/` | 사용자 모션과 저장 번들 | 각 프로젝트의 `project.json` |
| `presets/` | 전역 사용자 부위 그룹 프리셋 | `groups.json` |
| `tasks/` | CPU 상자 태스크 정의와 실행 결과 | `task.json`, `runs/*` |
| `integrations/` | 외부 모델의 고정 파라미터·해시·출처 | 각 JSON manifest/profile |
| `external/` | 설치 스크립트가 준비하는 외부 저장소·모델; 대개 Git 제외 | 통합 문서 참조 |
| `scripts/` | 설치, 검증, Blender 자산 변환 | `test.sh`, `setup-*.sh` |
| `tests/` | Python 계약·회귀 테스트 | 기능명과 대응하는 `test_*.py` |
| `docs/` | 기능별 상세 배경과 실험 기록 | 이 문서 하단의 관련 문서 표 |

`motions/`, `tasks/`, `presets/groups.json`, 미추적 `assets/`는 사용자 생성 데이터일 수 있다. 기능 변경 시 임의로 정리하거나 테스트 fixture로 덮어쓰면 안 된다.

## 6. 로봇 모델과 상태 표현

### 6.1 모델 변형

- `g1`: `assets/g1/g1.xml`의 기본 29-DoF G1.
- `g1-tools`: 기본 MJCF에 왼쪽 end-support와 오른쪽 scoop 자산을 붙인 모델. 편집 핸들은 손목이 아니라 각 도구의 TCP를 가리킨다.

모델 파일의 SHA-256은 프로젝트 호환성 판단에 사용한다. 과거에 허용한 fingerprint도 명시적으로 유지하며, 관절 이름과 순서가 다르면 프로젝트를 거부한다. 즉, 단순히 `nq=36`만 같다고 호환으로 보지 않는다.

소스 레퍼런스:

- `motioncreator/robot.py` — `Robot.__init__`, `Robot.fingerprint`, `Robot.compatible_fingerprints`
- `motioncreator/tool_model.py` — `tool_model_xml`
- `motioncreator/server.py` — `select_model`, `visual_response`

### 6.2 qpos와 화면 상태

`qpos`는 `[root xyz(3), root quaternion wxyz(4), 29 joint angles]`의 36개 값이다. Three.js가 쓰는 quaternion은 `xyzw`이므로 API 화면 상태는 핸들/geom 방향을 `xyzw`로 내보내지만 프로젝트 루트 `qpos`는 MuJoCo의 `wxyz`를 유지한다.

`Robot.state(q)`는 프런트엔드 렌더링에 필요한 다음 정보를 계산한다.

- 정규화·제한 검사된 `qpos`.
- 모든 편집 핸들의 월드 위치·방향·한글 레이블.
- 시각 geom별 월드 pose.
- 파지 패드의 pose와 크기.
- 질량중심과 최저점의 바닥 여유.
- 개별 힌지의 현재 각도, 범위, 월드 축, 피벗 위치.

소스 레퍼런스:

- `motioncreator/robot.py` — `Robot.validate_q`, `Robot.data`, `Robot.point`, `Robot.state`
- `frontend/lib/robot-scene.ts` — `PoseState`, `RobotScene.update`

### 6.3 핸들 계층

기본 부위 핸들은 골반, 양손, 양발, 양무릎, 양팔꿈치, 양어깨다. MJCF의 모든 29개 hinge joint에서도 관절 핸들을 자동 생성한다. 고관절·허리·발목은 UI 편의를 위해 여러 관절을 하나의 결합 컨트롤로 노출한다.

- 고관절: yaw/pitch/roll 3축, roll anchor 위치에서 회전.
- 허리: yaw/roll/pitch 3축, torso 방향을 목표로 계산.
- 발목: roll/pitch 2축의 실제 모터 각도. 임의 3D orientation 목표가 아니다.
- 팔꿈치·무릎과 개별 관절: 실제 hinge 축 하나를 사용한다.

소스 레퍼런스:

- `motioncreator/robot.py` — `HANDLES`, `JOINT_HANDLES`, `HINGES`, `ANGLE_LOCKABLE`
- `frontend/lib/robot-scene.ts` — `COMBINED_JOINTS`, `jointForRing`, `ankleFrame`
- `frontend/lib/body-groups.ts` — `BODY_GROUPS`, `controlSelection`, `controlKey`

## 7. 전신 IK 설계

`Robot.solve`는 SciPy `least_squares`로 자유 루트와 29개 관절을 joint limit 안에서 최적화한다. 요청은 위치 목표, orientation 목표, 직접 joint 목표, 위치 pin, 방향/관절각 pin을 함께 포함할 수 있다. 해가 나왔다고 무조건 채택하지 않고 목표 오차와 고정 오차를 평가해 `solver.rejected`와 mm 단위 진단을 돌려준다.

### 7.1 제약의 의미

- `targets`: 핸들의 목표 월드 위치.
- `orientation_targets`: 골반·손·발·고관절·허리 등 허용된 핸들의 목표 방향.
- `joint_targets`: 개별 hinge 또는 결합 컨트롤의 관절각.
- `pins`: 핸들의 월드 위치 유지. 발 pin은 방향도 함께 유지한다.
- `angle_pins`: 위치는 허용하되 핸들 방향 또는 hinge 각도만 유지한다.
- `anchor`: 움직임 전 자세. elastic 모드의 자세 기준점이 된다.
- `resistance`: 목표에서 먼 관절이 원래 자세에 머무는 경향의 강도.
- `mode=elastic`: 선택 부위에서 관절 그래프상 먼 자유도에 더 큰 자세 유지 가중치.
- `mode=free`: 고정과 목표를 제외한 자세 유지 제약을 약하게 한다.

프런트엔드는 드래그 중 요청을 직렬화한다. 최신 목표 하나만 `pending`에 남겨 서버 응답보다 마우스 이벤트가 빠를 때 오래된 IK 요청이 쌓이지 않게 한다.

소스 레퍼런스:

- `motioncreator/robot.py` — `Robot.solve`, `right_jacobian`, `left_jacobian_inverse`
- `motioncreator/server.py` — `SolveInput`, `GroupSolveInput`, `solve`, `solve_group`
- `frontend/app/page.tsx` — `begin`, `drain`, `move`, `rotate`, `jointAngle`

### 7.2 선택·다중 이동·미러

부위 그룹 이름을 누르면 하위 핸들을 모두 선택하고, Shift 선택은 기존 집합에 추가·해제한다. 다중 위치 이동은 선택 중심에 기즈모를 놓고 모든 목표에 같은 delta를 적용한다. 좌우 미러 모드는 골반의 현재 좌우 대칭면을 기준으로 좌우 짝의 Y 성분을 반사하며, 현재 자세 자체를 강제로 대칭화하지 않는다.

소스 레퍼런스:

- `frontend/lib/body-groups.ts` — `selectMembers`, `selectionState`, `expandVirtualControls`
- `frontend/lib/pose-transforms.ts` — `counterpart`, `canMirrorSelection`, `translatedTargets`
- `frontend/app/page.tsx` — `applySelection`, `selectBatch`, `selectionCenter`

### 7.3 회전

손·발·골반 등은 world/local 회전 기즈모를 지원한다. 그룹 회전은 기준 핸들의 orientation을 축으로 선택 중심 주변의 위치와 방향을 함께 회전시킨다. hinge는 기즈모 quaternion 변화에서 실제 축의 twist만 누적하고 관절 한계로 clamp한다.

소스 레퍼런스:

- `frontend/lib/pose-transforms.ts` — `rotatedGroupTargets`, `incrementRotation`, `quaternionFromDegrees`
- `frontend/lib/robot-scene.ts` — `RobotScene.applyHingeDrag`, `RobotScene.setTransformMode`
- `frontend/app/page.tsx` — `numericRotate`, `nudgeRotation`, `applyHingeAngle`

## 8. 메인 편집기 기능

### 8.1 초기화와 브라우저 초안

`GET /api/init`가 기본 pose, 새 프로젝트, 관절 이름·범위, 모델 id와 시각 자산 revision을 반환한다. 브라우저는 모델별 `localStorage` 초안을 검증 API에 통과시킨 뒤 복원한다. 편집 중 project/current qpos/pin/scene 상태는 500 ms debounce로 다시 저장하지만, 이는 디스크의 `motions/` 저장을 대신하지 않는다.

로봇 모델을 바꿀 때는 현재 초안을 먼저 모델별 key에 저장하고 서버 모델 변경 후 페이지를 새로고침한다.

소스 레퍼런스:

- `motioncreator/server.py` — `initialize`, `select_model`
- `frontend/app/page.tsx` — `Editor`, `switchModel`, 초기화 `useEffect`

### 8.2 Three.js 장면

`RobotScene`은 로봇 GLB를 불러와 `geom_*` 노드를 API pose에 맞춰 갱신한다. 편집용 로봇과 반투명 reference ghost를 별도로 유지한다. 마커 선택, 카메라, COM 바닥 링, 변환 기즈모, grip pad, 장면 물체, 파지면 raycast를 모두 한 클래스가 담당한다.

주요 조작:

- `W`: 이동, `E`: 회전, `F`: 선택 부위에 카메라 맞춤.
- 빈 공간 왼쪽 드래그: 회전, 우클릭 드래그: 이동, 휠: 확대.
- `Ctrl+Z`, `Ctrl+Shift+Z`: 자세·pin·장면 오브젝트 변경 undo/redo.
- 물리 재생 중에는 로봇 편집과 키보드 조작을 잠근다.

소스 레퍼런스:

- `frontend/lib/robot-scene.ts` — `RobotScene.constructor`, `handleKeyDown`, `focusSelection`, `select`, `update`
- `frontend/app/page.tsx` — `checkpoint`, `changeHistory`

### 8.3 부위 그룹과 프리셋

기본 그룹 트리는 코드에 고정되어 항상 제공된다. 사용자 프리셋은 선택한 핸들 id 목록만 `presets/groups.json`에 저장하며 pose, pin, 관절각을 저장하지 않는다. 저장은 임시 파일을 거친 원자적 교체 방식이고 이름·멤버 유효성을 검사한다.

소스 레퍼런스:

- `frontend/lib/body-groups.ts` — `BODY_GROUPS`, `allNodes`
- `motioncreator/presets.py` — `GroupStore`, `GroupStore.save`, `GroupStore.delete`
- `motioncreator/server.py` — `list_groups`, `save_group`, `delete_group`

### 8.4 실행 취소 범위

undo snapshot에는 현재 qpos, pin, angle pin, pose dirty 여부, 키프레임, 장면 물체와 그룹이 포함된다. 오브젝트 기즈모 드래그는 drag 시작 시 한 번만 checkpoint한다. 재생·계산 중이거나 텍스트 입력 자체의 브라우저 undo가 필요한 상황은 편집기 undo에서 제외한다.

소스 레퍼런스:

- `frontend/app/page.tsx` — `editorSnapshot`, `checkpoint`, `changeHistory`
- `frontend/lib/robot-scene.ts` — `handleKeyDown`

## 9. 프로젝트와 키프레임 데이터 모델

### 9.1 Project 핵심 필드

```jsonc
{
  "format": "motioncreator.g1.v1",
  "project_id": "32 lowercase hex",
  "created_at": "ISO 8601 with timezone",
  "name": "사용자 이름 또는 빈 문자열",
  "name_mode": "manual | auto",
  "model_sha256": "MJCF fingerprint",
  "joint_names": ["29개 관절명"],
  "coordinate_system": "right-handed, +X forward, +Y left, +Z up",
  "units": {"position": "m", "angle": "rad", "time": "s"},
  "keyframes": [],
  "current_qpos": ["선택적 현재 편집 자세"],
  "pins": ["현재 편집 pin"],
  "angle_pins": ["현재 편집 angle pin"],
  "scene_objects": [],
  "scene_groups": []
}
```

`project_id`는 같은 프로젝트의 반복 저장 폴더를 찾는 안정된 identity다. 레거시 프로젝트를 서버의 저장 목록에서 열면 경로 기반 UUID를 부여하고, 일반 파일 업로드에서 id가 없으면 새 UUID를 부여한다. 자동 이름은 일반적인 프레임명을 제외한 최대 3개 키프레임 이름과 생성일을 조합한다.

소스 레퍼런스:

- `motioncreator/motion.py` — `new_project`, `ensure_project_identity`, `prepare_saved_project`, `automatic_project_name`

### 9.2 Keyframe

일반 키프레임은 다음 의미를 가진다.

- `name`: 사용자 의미를 담는 이름.
- `qpos`: 해당 지점의 정확한 로봇 상태.
- `duration`: **이전 키프레임에서 이 키프레임에 도달하는 시간**. 첫 프레임은 보간 구간이 없으므로 일반 모션에서 적용되지 않는다.
- `pins`: 구간 양 끝에 공통으로 있을 때 보간 중 유지할 위치 고정.
- `angle_pins`: 구간 양 끝에 공통으로 있을 때 유지할 방향·관절각 고정.
- `grasp`: 해당 키프레임의 물체·접촉면·힘 검증 설정.

가져온 NPZ/CSV는 모든 샘플을 `samples`에 보존한 **단일 모션 클립 키프레임**이 된다. 이 모드에서는 다른 키프레임, pin, grasp를 함께 둘 수 없다. duration을 바꾸면 전체 클립의 재생 속도를 바꾼다.

소스 레퍼런스:

- `frontend/lib/keyframes.ts` — `Keyframe`, `duplicateKeyframeAfter`
- `motioncreator/motion.py` — `validate_project`, `project_from_motion_bytes`, `compile_motion_clip`

### 9.3 검증 한계

- 키프레임 1–100개, 총 프로젝트 길이 최대 600초.
- 일반 키프레임 duration 0.1–60초.
- 클립 샘플 1–3000개, 클립 duration `1/120`–600초.
- 장면 물체 최대 32개, 장면 그룹 최대 16개.
- qpos 유한값, quaternion 정규화, joint limit, 모델 fingerprint와 관절 순서를 검사한다.
- 장면 id, 크기, mass, friction, opacity, imported asset metadata를 검사한다.
- pin된 두 endpoint의 위치·방향 또는 angle 차이가 허용 오차보다 크면 컴파일을 거부한다.

소스 레퍼런스:

- `motioncreator/motion.py` — `MAX_KEYFRAMES`, `MAX_MOTION_SAMPLES`, `validate_project`

## 10. 모션 컴파일과 재생

일반 키프레임 컴파일 순서:

1. 프로젝트 전체 계약을 검증한다.
2. 각 구간의 endpoint qpos와 핸들 FK를 구한다.
3. quaternion 부호를 연속화한다.
4. destination frame의 duration과 FPS로 샘플 수를 정한다.
5. `s=u³(10-15u+6u²)` quintic easing으로 루트 위치와 관절각을 보간한다.
6. 루트 방향은 최단 경로 quaternion 보간으로 만든다.
7. 공통 pin/angle pin이 있으면 각 중간 샘플을 짧은 IK로 다시 projection한다.
8. MuJoCo `mj_differentiatePos`와 수치 gradient로 qvel/qacc를 계산한다.

기구학 미리보기는 최대 30 FPS로 컴파일한 상태 배열을 브라우저가 `requestAnimationFrame`으로 재생한다. 이 결과의 contact는 실제 힘 측정이 아니라 저작한 발 지지 플래그다.

소스 레퍼런스:

- `motioncreator/motion.py` — `compile_motion`, `motion_result`, `compile_motion_clip`
- `motioncreator/server.py` — `preview`
- `frontend/app/page.tsx` — `play`, 재생 `useEffect`

## 11. 장면 물체와 외부 3D 모델

### 11.1 기본 도형

지원 shape는 `box`, `open_box`, `sphere`, `cylinder`다. 각 물체는 id, 이름, pose, 크기, 질량, 마찰, 색, 투명도, visibility, fixed와 배치 옵션을 가진다.

- `ground_lock`: 회전·크기 변경 후 최저점을 지면에 맞춘다.
- `surface_snap`: 2 cm 이내의 옆면을 맞닿게 한다.
- `prevent_overlap`: 회전된 물체의 AABB가 다른 물체와 겹치지 않게 이동을 보정한다.
- 그룹: 여러 물체의 상대 위치·방향을 유지한 채 공통 피벗으로 이동·회전한다. 한 물체는 한 그룹에만 속한다.

배치 보정은 authoring 편의 기능이며 정확한 mesh-level collision solver가 아니다. 실제 물리 충돌은 Python이 생성한 MuJoCo geom을 따른다.

소스 레퍼런스:

- `frontend/lib/scene-objects.ts` — `SceneObject`, `placeSceneObject`, `createSceneObject`, `transformSceneObjectGroup`
- `frontend/app/scene-object-controls.tsx` — `SceneObjectControls`
- `frontend/app/page.tsx` — `commitObjects`, `createObjectGroup`, `changeObjectGroup`

### 11.2 GLB 가져오기

현재 API는 GLB만 수용한다. 업로드 내용의 SHA-256을 asset id로 사용하므로 동일 파일은 같은 자산을 재사용한다. trimesh로 geometry와 bounds를 검사하고, 정규화된 렌더링 GLB 및 `metadata.json`을 `assets/scene-models/<digest>/`에 저장한다.

여러 오브젝트/파트가 있으면 각 파트를 별도 편집 물체로 만들 수 있다. 화면은 원본 GLB를 bounds와 axis transform으로 정규화하지만, 물리는 shape와 크기 기반 프록시를 쓴다. `open_box`는 바닥과 네 벽의 box geom으로 구성한다. 따라서 보이는 복잡한 메시와 물리 충돌 형상은 같지 않을 수 있다.

소스 레퍼런스:

- `motioncreator/scene_assets.py` — `import_scene_asset`, `_asset_parts`, `asset_path`, `asset_part_path`
- `motioncreator/scene_geometry.py` — `append_collision_geoms`, `grounded_position`
- `frontend/lib/scene-objects.ts` — `normalizedSceneAsset`, `createImportedSceneObjects`
- `frontend/lib/robot-scene.ts` — `RobotScene.setSceneObjects`

## 12. 양손 파지 저작과 검증

### 12.1 파지 데이터

파지는 editable keyframe에만 붙는다. 사용자는 primitive box의 왼쪽·오른쪽 표면을 클릭하거나 양면 중앙을 선택한다. 접촉점은 물체 로컬 UV와 normal로 저장되므로 물체 pose를 따라간다. fit 시점의 물체 pose는 키프레임 전용 `object_pose`로 저장되어, 이후 전역 장면 물체가 움직여도 해당 키프레임의 grasp ghost를 독립 편집할 수 있다.

대표 필드:

- `object_id`, `object_pose`, `follow_object`.
- `left_surface_uv`, `right_surface_uv`.
- 손바닥 간격/닫기 시간과 손목 비틀림 후보.
- 검증 최소 정상력 `target_force_n`과 안전 상한 `max_force_n`.
- fit 결과의 `closure_qpos`, 접촉점 등 파생 정보.

소스 레퍼런스:

- `frontend/lib/keyframes.ts` — `TwoHandGrasp`
- `frontend/app/grasp-controls.tsx` — `GraspControls`, `freshGrasp`
- `frontend/app/page.tsx` — `pickGripSurface`, `fitGrasp`, `graspGhost`

### 12.2 자세 맞추기

서버는 물체 로컬 접촉점을 월드 좌표로 바꾸고, 시각적인 손목 중심이 아니라 rubber-hand의 안쪽 지지 패드가 목표 면에 닿도록 손목 pose 후보를 만든다. 원래 키프레임 qpos를 anchor로 양팔 orientation/position IK를 풀고, 허용 가능한 손목 twist 후보 중 오차와 자세 변화가 좋은 것을 선택한다. fit은 저작 자세를 만들 뿐 물체를 붙이거나 압착력을 보장하지 않는다.

소스 레퍼런스:

- `motioncreator/grasp.py` — `fit_two_hand_grasp`, `validate_grasp_event`
- `motioncreator/grip_geometry.py` — `grip_pad_center`, `grip_pad_contact_anchor`, `grip_pad_rotation`
- `motioncreator/server.py` — `grasp_fit`

### 12.3 물리 검증

선택 키프레임부터 물리 미리보기를 실행하면 설정한 닫기 구간을 거친 뒤 이후 모션을 추종한다. 손–물체 contact normal force, 양손 동시 접촉, 목표 힘 도달, 상한 초과, 최대 침투를 기록한다. 기본 G1은 rubber-hand 충돌체를 추가하고 도구 모델은 실제 도구 collision geom을 사용한다. 물체는 free body이며 `fixed=true`인 경우에만 world weld를 사용한다.

소스 레퍼런스:

- `motioncreator/policy_preview.py` — `compile_preview_motion`, `build_model`, `simulate.update_grasp_stats`
- `motioncreator/hand_collision.py` — `physical_hand_geom_names`
- `frontend/app/page.tsx` — `validateGraspPhysics`

## 13. Physics / GEAR-SONIC 미리보기

세 가지 재생 모드가 있다.

| 모드 | 계산 |
|---|---|
| Physics OFF | `compile_motion` 결과를 그대로 재생하는 기구학 미리보기 |
| Physics ON, SONIC OFF | 자유 루트 MuJoCo에서 29개 joint reference를 PD로 추종 |
| Physics ON, SONIC ON | SONIC ONNX가 50 Hz로 joint target을 생성하고 PD torque로 적용 |

공통 물리 특성:

- MuJoCo 500 Hz, 목표/정책 50 Hz.
- 중력, 바닥, 토크 제한, 동적 장면 물체를 사용한다.
- 루트 teleport·고정·외부 지지력을 사용하지 않는다.
- 선택한 키프레임을 초기 상태로 하고 그 이후만 계산할 수 있다.
- 60초 이하, 동시에 한 작업만 실행, 최근 결과 최대 4개 유지.
- `fallen`, 수치 불안정, joint RMSE, torque saturation 등을 요약한다.
- 결과는 임시 replay이며 원본 project/keyframes를 바꾸지 않는다.

PD 모드는 균형 정책이 아니므로 모터가 움직여도 넘어질 수 있다. SONIC 모드는 original-release G1 reference 계약만 지원하며 `.conda-policy`와 고정된 ONNX 자산이 필요하다. encoder/decoder 세션은 상주 worker에서 재사용하지만 rollout history는 작업마다 reset한다.

소스 레퍼런스:

- `motioncreator/policy_preview.py` — `runtime`, `build_model`, `simulate`, `PreviewJobs`
- `motioncreator/sonic.py` — `Reference`, `SonicCPU`, `SonicCPU.action`
- `motioncreator/server.py` — `policy_preview`, `policy_status`, `policy_result`, `policy_cancel`
- `frontend/app/page.tsx` — `play`, `cancelPolicy`, `changeSimulation`

## 14. 저장, 가져오기, 내보내기

### 14.1 프로젝트 번들

기본 저장 결과:

```text
motions/<display-name>_<identity>/
  project.json      # 편집 가능한 원본, 키프레임·장면·파지 포함
  motion.npz        # Kimodo G1Skeleton34 5-array 계약
  motion.csv        # 샘플별 MuJoCo qpos 36열
  environment.json  # 좌표계, 단위, 중력, 바닥, 장면 물체·그룹
  metadata.json     # 모델/형식/보간/검증 의미와 environment SHA-256
```

모션 산출물은 임시 파일에 모두 쓴 뒤 `os.replace`로 각 파일을 원자적으로 갱신한다. 같은 `project_id`를 다시 저장하면 기존 폴더를 갱신하며, 이름이 바뀌면 같은 폴더를 rename한다. `save-as`는 id와 생성 시간을 새로 만들어 복사본을 만든다.

`project.json`의 `current_qpos`는 현재 화면 자세를 보존하지만, `motion.npz/csv`는 오직 timeline keyframe으로 컴파일한다. 따라서 화면 자세 변경 후 키프레임에 반영하지 않으면 출력 모션에는 포함되지 않는다.

소스 레퍼런스:

- `motioncreator/motion.py` — `save_bundle`, `_existing_project_folder`, `_atomic_bytes`, `environment_snapshot`
- `motioncreator/server.py` — `save`, `save_as`, `download`, `saved`, `open_saved_project`
- `frontend/app/page.tsx` — `exportProject`, `loadProject`, `openFile`

### 14.2 Kimodo NPZ

NPZ에는 정확히 다음 5개 배열이 들어간다.

- `posed_joints`: `[T, 34, 3]`
- `global_rot_mats`: `[T, 34, 3, 3]`
- `local_rot_mats`: `[T, 34, 3, 3]`
- `root_positions`: `[T, 3]`
- `foot_contacts`: `[T, 4]`, bool

Kimodo 좌표계는 `+Z forward, +Y up`이므로 MuJoCo 저작 좌표계와 명시적 축 변환을 한다. NPZ 자체에는 FPS가 없고 metadata가 이를 보완한다. 브라우저에서 외부 NPZ/CSV를 가져올 때는 현재 선택한 FPS를 원본 샘플 간격으로 해석한다.

`foot_contacts`는 측정 force가 아니라 발 속도와 높이에 따른 Kimodo geometric heuristic이다. 저작 pin/contact semantics와 혼동하면 안 된다.

소스 레퍼런스:

- `motioncreator/kimodo_format.py` — `export_kimodo_g1`, `validate_kimodo_g1`, `kimodo_g1_to_qpos`
- `motioncreator/motion.py` — `project_from_motion_bytes`
- `motioncreator/reference.py` — `load_reference`, `_load_kimodo_reference`

### 14.3 ProtoMotions

ProtoMotions 저장 옵션은 기본 NPZ 저장이 성공한 뒤 별도 `.conda-protomotions` 프로세스에서 파생 파일을 만든다. 프로필에 고정한 upstream commit, 대상 MJCF SHA-256, 소스 모델 fingerprint를 모두 검사한다. 관절은 배열 위치를 가정하지 않고 이름으로 permutation하며 대상 모델에서 FK와 velocity를 다시 계산한다.

출력은 `.motion`, `.pt`, `.protomotions.json`이다. 원본 root translation을 자동 ground-shift하지 않는다. 변환 실패는 이미 저장된 편집 원본을 삭제하지 않고 warning으로 반환한다.

소스 레퍼런스:

- `motioncreator/protomotions_bridge.py` — `export_isolated`, `convert_reference`
- `motioncreator/reference.py` — `joint_permutation`
- `integrations/protomotions-profile.json` — 검증된 revision과 모델 계약

## 15. Decoupled WBC 워크벤치

이 페이지는 메인 편집기가 만든 project를 다시 컴파일한 뒤, 제어권을 다음처럼 나눈다.

- 양팔 14 DoF: 저작 joint trajectory를 직접 PD 추종.
- 허리 + 양다리 15 DoF: GR00T Decoupled WBC 정책 출력.
- 저작 root trajectory: auto 모드에서 현재 로봇과 시작점을 정렬한 뒤 local 전후/좌우 속도, yaw rate, body height 명령으로 변환. 로봇 root를 직접 쓰지 않는다.
- 저작 허리: yaw/roll/pitch 관절을 torso XYZ RPY 명령으로 변환하고 정책 한계로 clamp.

### 세션 생명주기

1. 저장 project 또는 JSON/NPZ/CSV를 불러온다.
2. `POST /api/decoupled-wbc/sessions`가 project 검증·컴파일·모델 생성을 수행한다.
3. 세션 thread가 200 Hz physics(`dt=0.005`)와 50 Hz policy/control을 실행한다.
4. WebSocket이 revision이 바뀐 snapshot만 화면에 보낸다. HTTP command/status도 fallback으로 제공한다.
5. Play는 simulation과 recording을 시작하고 Stop은 속도 명령을 0으로 만든다. Reset은 초기 상태와 녹화를 지운다.
6. 저장 시 50 Hz recording을 편집기로 다시 가져올 수 있는 번들로 기록한다.

auto와 manual 모드가 있다. manual은 `W/S`, `A/D`, `Q/E`, `1/2`, `3..8`, `Z`로 이동·높이·torso offset을 조정한다.

소스 레퍼런스:

- `motioncreator/decoupled_api.py` — `create_session`, `session_command`, `stream`, `save_session`
- `motioncreator/decoupled_wbc.py` — `CommandState`, `ghost_tracking_command`, `build_observation`, `LowerBodyPolicy`, `DecoupledSimulation`
- `frontend/app/decoupled-wbc-page.tsx` — `DecoupledWbcPage`

### 장면과 파지 피드백

저작한 장면 물체를 모두 free body로 WBC MJCF에 추가하며 `fixed=true`만 world weld한다. 손–물체, 물체–바닥, 물체–물체 contact pair를 명시적으로 만든다. 복잡한 imported mesh도 저장한 primitive proxy를 충돌체로 쓴다.

기본 G1의 편집 파지가 있으면 물체 질량, 보수적으로 clamp한 friction, 안전계수, 목표 상승 가속도로 손당 normal force 목표를 자동 계산한다. 접촉 후 force PI, 파지면 orientation PD, 물체 높이 지지 force를 관절 Jacobian transpose로 양팔 torque에 더한다. 자세 PD와 feedback torque의 합은 actuator limit으로 제한하며 관절별 기여와 포화율을 UI에 노출한다. 이 기능도 weld가 아니며 도구 모델에서는 현재 비활성화된다.

소스 레퍼런스:

- `motioncreator/decoupled_wbc.py` — `build_environment_model`, `automatic_grip_force`, `prepare_grasp_control`, `DecoupledSimulation._grasp_feedback_torque`
- `integrations/decoupled-wbc-parameters.json` — 제어 주기, gain, command/force limit

### 녹화 번들

`motions/Decoupled_WBC_*` 아래 `motion.npz`, `motion.csv`, `commands.csv`, `metadata.json`을 저장한다. motion은 현재 편집기에서 재가져올 수 있고, commands에는 navigation/height/torso input이 남는다. 최대 recording 시간은 파라미터 기준 300초다.

소스 레퍼런스:

- `motioncreator/decoupled_wbc.py` — `DecoupledSimulation.recording_copy`, `save_recording_bundle`

## 16. CPU 상자 태스크 PoC

이 흐름은 메인 키프레임 파지 검증과 별개다. `TaskSpec`으로 실제 상자, 인식 pose, 목적지, 마찰·질량·시간 제한을 정의하고 11단계 `approach → ... → retreat` 계획을 만든다.

실행 종류:

- `ik_preview`: 단계별 seed pose를 메인 `Robot.solve`로 연결한 빠른 기구학 참조.
- `ardy`: 외부 ARDY 생성기를 수치 조건으로 실행해 참조 NPZ를 생성.
- `simulate`: 생성 참조 또는 기존 NPZ를 SONIC/PD로 MuJoCo에서 추종.

각 실행은 `tasks/<task-id>/runs/<run-id>/` 아래 요청, 진행 상태, 참조, 장면 XML, contact log, simulation NPZ, replay와 보고서를 남긴다. 실행 worker는 `.conda-policy` subprocess이며 취소 파일을 감시한다. `status=completed`는 계산 완료일 뿐 성공이 아니며, 실제 성공은 `result.validated`로 판단한다.

현재 프런트 `TaskWorkbench`는 최상위 라우터에 연결되지 않았으므로, 이 API를 주 기능으로 가정해서 메인 `Editor`를 변경하면 안 된다.

소스 레퍼런스:

- `motioncreator/tasks.py` — `TaskSpec`, `plan_task`, `TaskFSM`
- `motioncreator/task_api.py` — task API router
- `motioncreator/task_jobs.py` — `save_task`, `start_run`, `get_run`
- `motioncreator/task_generation.py` — `ik_preview`, `generate_ardy`, `write_reference`
- `motioncreator/task_physics.py` — `build_scene`, `contact_metrics`, `simulate`
- `motioncreator/task_worker.py` — `main`
- `frontend/app/task-workbench.tsx` — `TaskWorkbench`

## 17. HTTP / WebSocket API 지도

### 메인 편집기

| API | 구현 함수 | 역할 |
|---|---|---|
| `GET /api/health` | `server.health` | 서버·모델 확인 |
| `GET /api/init` | `server.initialize` | 기본 pose/project/limit |
| `POST /api/model` | `server.select_model` | `g1`/`g1-tools` 변경 |
| `GET /api/robot/{model}.glb` | `server.model_visual` | 현재 모델 시각 자산 |
| `POST /api/pose` | `server.pose` | qpos FK 상태 |
| `POST /api/solve` | `server.solve` | 단일 목표 IK |
| `POST /api/solve-group` | `server.solve_group` | 위치·방향·joint 복합 IK |
| `POST /api/validate` | `server.validate` | 프로젝트 계약 검증·레거시 identity 보완 |
| `POST /api/import-motion` | `server.import_motion` | NPZ/CSV → clip project |
| `POST /api/preview` | `server.preview` | 기구학 모션 컴파일 |
| `POST /api/grasp-fit` | `server.grasp_fit` | 양손 파지 자세 계산 |
| `GET/POST/DELETE /api/groups` | `server.list_groups` 등 | 사용자 부위 프리셋 |
| `POST /api/scene-assets/import` | `server.import_scene_model` | GLB 가져오기 |
| `GET /api/scene-assets/*` | `server.scene_model*` | 가져온 GLB 제공 |
| `POST /api/save`, `/save-as` | `server.save`, `server.save_as` | 번들 저장 |
| `GET /api/saved` | `server.saved` | 저장 project 목록 |
| `GET /api/project/*` | `server.open_saved_project` | 저장 project 열기 |
| `GET /api/files/*` | `server.download` | 허용된 산출물 다운로드 |

### 물리 미리보기

| API | 구현 함수 | 역할 |
|---|---|---|
| `GET /api/policy-preview/runtime` | `server.policy_runtime` | PD/SONIC 가용성 |
| `POST /api/policy-preview` | `server.policy_preview` | 비동기 계산 시작 |
| `GET /api/policy-preview/{id}` | `server.policy_status` | 상태·진행률 |
| `GET /api/policy-preview/{id}/result` | `server.policy_result` | replay 결과 |
| `POST /api/policy-preview/{id}/cancel` | `server.policy_cancel` | 작업 취소 |

### Decoupled WBC

| API | 구현 함수 | 역할 |
|---|---|---|
| `GET /api/decoupled-wbc/runtime` | `decoupled_api.runtime` | 정책 asset 무결성 |
| `POST /api/decoupled-wbc/sessions` | `decoupled_api.create_session` | 세션 생성 |
| `GET/DELETE /api/decoupled-wbc/sessions/{id}` | `session_status`, `delete_session` | 조회·종료 |
| `POST .../{id}/command` | `session_command` | play/stop/reset/key/mode/force |
| `POST .../{id}/save` | `save_session` | 녹화 번들 저장 |
| `WS .../{id}/stream` | `stream` | command와 변경 snapshot 스트림 |

`/api/tasks/*`의 상세 계약은 `motioncreator/task_api.py`와 `docs/box-task-cpu.md`를 참고한다.

## 18. 안전성과 데이터 무결성 규칙

코드를 수정할 때 다음 규칙을 보존해야 한다.

1. 모델 일치: fingerprint와 joint names/order를 검증하지 않은 reference를 실행하지 않는다.
2. 단위·좌표계: qpos root quaternion은 wxyz, Three.js/object quaternion은 xyzw다. 변환 지점에서 순서를 명시한다.
3. pickle 금지: 외부 NPZ는 `allow_pickle=False`로 읽는다.
4. 경로 제한: 다운로드·프로젝트 열기는 절대 경로와 `..`를 거부하고 확장자를 allow-list한다.
5. 업로드 제한: motion은 100 MB 이하이고 지원 확장자만 허용한다. scene asset도 importer에서 크기·형식을 검사한다.
6. 원자적 저장: 사용자 project와 산출물은 임시 파일 후 replace한다.
7. 원본 보존: ProtoMotions 파생 변환 실패가 native 저장 성공을 되돌리면 안 된다.
8. 물리 의미: kinematic preview, PD physics, policy physics, 측정 contact를 UI·metadata에서 구분한다.
9. 파지 의미: 손–물체 attachment를 암묵적으로 추가하지 않는다. `fixed` 물체 외의 weld는 금지된 가정이다.
10. 생성 데이터: 사용자가 저장한 `motions/`, `tasks/`, preset, 미추적 asset을 테스트나 정리 과정에서 수정·커밋하지 않는다.

소스 레퍼런스:

- `motioncreator/motion.py` — `validate_project`, `_atomic_bytes`, `save_bundle`
- `motioncreator/reference.py` — `load_reference`
- `motioncreator/server.py` — `import_motion`, `download`, `open_saved_project`
- `motioncreator/scene_assets.py` — `import_scene_asset`, `_validate_identifier`

## 19. 테스트와 검증

### 기본 검증

```bash
./scripts/test.sh
npm --prefix frontend run build
npm --prefix frontend run test:scene
```

- `scripts/test.sh`: 격리된 `.conda`로 `tests/` 전체 pytest.
- 프런트 build: TypeScript type check 후 Vite production build.
- `test:scene`: 브라우저 없이 body group, pose transform, scene placement 등 TS 모듈을 Node test로 검증.

### 선택적 통합 검증

```bash
./scripts/test-task-cpu.sh       # .conda-policy, ARDY/SONIC 자산 필요
./scripts/test-protomotions.sh   # .conda-protomotions와 upstream 필요
```

테스트 탐색 지도:

| 변경 영역 | 우선 테스트 |
|---|---|
| IK·pin·회전 | `test_motion.py`, `test_rotation.py`, `test_all_joints.py`, `test_hip_controls.py`, `test_waist_ankle_controls.py` |
| 키프레임·저장·가져오기 | `test_motion.py`, `test_reference.py` |
| 파지 | `test_grasp.py` |
| 물리/SONIC | `test_policy_preview.py`, `test_task_cpu.py` |
| 장면·GLB | `test_scene_assets.py`, `frontend/tests/robot-scene.test.mjs` |
| 모델 변경·도구 | `test_model_selection.py`, `test_tool_model.py` |
| WBC | `test_decoupled_wbc.py` |
| CPU 태스크 | `test_tasks.py`, `test_task_api.py`, `test_task_download.py` |
| ProtoMotions | `test_protomotions.py` |

## 20. AI 에이전트용 빠른 읽기 순서

목적별 최소 읽기 경로는 다음과 같다.

### 메인 편집 동작을 바꿀 때

1. `frontend/app/page.tsx` — `Editor`: 상태 소유권과 API 호출.
2. `frontend/lib/robot-scene.ts` — `RobotScene`: 화면 이벤트와 기즈모.
3. `motioncreator/server.py` — 해당 endpoint.
4. `motioncreator/robot.py` — `Robot.solve` 또는 `Robot.state`.
5. 대응 테스트.

### 저장 형식·타임라인을 바꿀 때

1. `motioncreator/motion.py` — `validate_project`, `compile_motion`, `save_bundle`.
2. `motioncreator/kimodo_format.py` — NPZ 계약.
3. `motioncreator/reference.py` — downstream 로딩 계약.
4. `frontend/lib/keyframes.ts`와 `frontend/app/page.tsx`.
5. `test_motion.py`, `test_reference.py`, 필요 시 `test_protomotions.py`.

### 장면·파지를 바꿀 때

1. `frontend/lib/scene-objects.ts`.
2. `motioncreator/motion.py` — scene validation.
3. `motioncreator/scene_geometry.py`와 `scene_assets.py`.
4. `motioncreator/grasp.py`, `grip_geometry.py`.
5. `policy_preview.build_model`과 `decoupled_wbc.build_environment_model`을 함께 확인한다. 같은 장면 데이터가 여러 물리 경로에서 사용되기 때문이다.

### 정책·물리를 바꿀 때

1. `motioncreator/policy_preview.py` — 일반 physics preview.
2. `motioncreator/sonic.py`와 `integrations/sonic-parameters.json`.
3. `motioncreator/decoupled_wbc.py` — WBC는 별도 정책·제어 계약임을 유의.
4. CPU 태스크까지 영향이 있으면 `task_physics.py`도 확인한다.

## 21. 변경 시 자주 놓치는 연결점

- `Robot.handles`를 바꾸면 Python의 `HANDLES`뿐 아니라 `BODY_GROUPS`, `RobotScene` 회전 가능 목록, project validation, 테스트가 함께 영향을 받는다.
- scene object 필드를 추가하면 TypeScript type/default/migration, `validate_project`, `environment_snapshot`, 두 물리 모델 builder를 모두 갱신해야 한다.
- qpos나 joint order를 바꾸면 기존 project fingerprint, Kimodo mapping, SONIC permutation, ProtoMotions profile, Decoupled WBC observation이 모두 깨질 수 있다.
- 새로운 저장 확장자는 `/api/files` allow-list에 추가하지 않으면 UI 다운로드가 실패한다.
- `project.current_qpos`는 timeline source가 아니다. 저장 모션을 바꾸려면 keyframe을 갱신해야 한다.
- 프런트 `TaskWorkbench`가 존재해도 현재 사용자 라우트에 연결되어 있지 않다.
- PD 물리 성공은 정책 추종 성공이나 동적 안정성 보장이 아니다. metadata와 UI 문구에서 검증 범위를 과장하지 않는다.
- visual GLB와 MuJoCo collision은 서로 다른 표현일 수 있다. imported GLB의 렌더 bounds와 collision proxy를 혼동하지 않는다.
- `g1-tools`는 단순 시각 스킨이 아니라 MJCF, handle/TCP, fingerprint, collision이 다른 모델이다.

## 22. 알려진 한계

- 전신 IK는 수치 최적화이며 전역 해, 충돌 회피, 동적 균형을 보장하지 않는다.
- 저작 중 COM과 바닥 높이는 기구학 정보다. 안정성은 physics/policy 실행으로 별도 평가해야 한다.
- 기본 장면 도형 배치의 겹침 방지는 AABB 기반이다.
- imported GLB 물리는 원본 triangle mesh가 아니라 primitive/open-box proxy다.
- 파지는 강체 물체와 근사 손 충돌체를 사용하며 재료 변형, 손가락 제어, 실제 force sensor loop를 모델링하지 않는다.
- SONIC/Decoupled WBC 결과는 고정된 모델·파라미터·CPU adapter에 대한 시뮬레이션 결과이며 실기 안전을 보장하지 않는다.
- CPU 상자 태스크 PoC의 기본 팔레타이징 성공은 기존 기록상 완전히 검증되지 않았다.
- 브라우저 초안은 로컬 저장소 용량과 브라우저 프로필에 종속된다.

## 23. 관련 상세 문서

| 문서 | 내용 |
|---|---|
| `README.md` | 사용자 중심 실행·편집 절차와 현재 기능 소개 |
| `docs/physics-validation-plan.md` | 정밀 자세 편집과 물리 검증의 단계별 의미 |
| `docs/protomotions-integration.md` | Kimodo/ProtoMotions 형식, 모델 검증, sim2sim 경계 |
| `docs/decoupled-wbc.md` | Decoupled WBC 사용법과 설치 |
| `docs/box-task-cpu.md` | ARDY/SONIC CPU 상자 태스크 PoC 상세 |
| `docs/scene-model-import.md` | GLB/BLEND 자산과 좌표·충돌체 설계 |
| `docs/tool-model.md` | G1 도구 장착 모델과 라면 장면 |
| `docs/mimickit-review.md` | G1 물리 모델과 MimicKit 적합성 검토 |

이 문서와 코드가 충돌하면 실행 소스와 테스트를 우선한다. 기능을 추가하거나 계약을 바꾸면 해당 구현의 `파일 — 함수/클래스` 레퍼런스와 데이터 불변조건도 이 문서에 함께 갱신한다.
