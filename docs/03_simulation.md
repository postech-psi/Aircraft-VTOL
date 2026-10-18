# 3. 시뮬레이션

PX4 v1.17.0 SITL과 Gazebo Harmonic으로 기체 한 대가 임무 하나를 비행하게 합니다. PX4는 실제 펌웨어
그대로이고, 이 저장소의 C++ 플러그인이 날개·동체 공력과 추진 손실을 더합니다. 현재 모델 이름은
`aeroprop-v2`입니다.

가정한 기체가 어떤 조건에서 제어 여유를 잃는지 찾는 데 씁니다. 모델 계수의 대부분이 가정값이고 실측과
비교한 적이 없으므로, 시뮬레이션에서 통과했다고 실기체가 그 조건에서 날 수 있다는 뜻은 아닙니다. 최근
실행 결과는 [시뮬레이션 보고서](../missions/2026_year_end/report.md)에 있습니다.

## 1. 실행하기

시뮬레이션은 Docker 안에서 돌립니다. 이미지에 Ubuntu 24.04, Gazebo Harmonic, PX4 v1.17.0 빌드가 들어
있어서 호스트에는 Docker만 있으면 됩니다. 이미지 빌드 옵션과 custom PX4 연결은
[simulation/README.md](../simulation/README.md)를 보세요. repo 최상위에서 실행합니다.

```bash
docker build -f simulation/Dockerfile -t psi-vtol-sitl:v1.17.0-harmonic simulation
bash simulation/docker_run.sh cmake -S simulation/gazebo -B simulation/gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
bash simulation/docker_run.sh cmake --build simulation/gazebo/build -j2
bash simulation/docker_run.sh python3 -m pytest -q
bash simulation/docker_run.sh
```

`docker_run.sh`는 repo를 `/workspace`에 마운트한 새 컨테이너에서 명령을 실행하고, 인자가 없으면 기준
기체로 임무를 한 번 비행합니다. 공력 플러그인을 빌드하지 않았으면 runner가 시작하지 않습니다. Python은
마운트로 바로 반영되지만 C++을 고쳤다면 플러그인을 다시 빌드해야 합니다.

값을 바꿔서 비행하려면 `--set`을 씁니다. 바꿀 수 있는 이름은 `vehicle/psi_qp.yaml`, `study.yaml`의
`environment`, `mission.yaml`의 `profile`에 있는 키입니다. PX4 파라미터는 `--px4`로 바꿉니다.

```bash
bash simulation/docker_run.sh python3 -m psi_vtol.sim.run_case --set wind_speed_mps=4 --set wind_from_deg=90
bash simulation/docker_run.sh python3 -m psi_vtol.sim.run_case --px4 MC_YAW_P=0.3
```

`--set`으로 기체 값을 바꿔도 PX4에 알려 주는 모터 기하는 기준값 그대로입니다. 실제 기체가 PX4가 아는
것과 다른 상황을 흉내 내기 위해서입니다.

비행 중에 일이 생기게 하려면 `--event 이름@단계+초`를 씁니다. 그 단계에 들어가고 정한 초가 지나면(단계가
그보다 짧으면 단계가 끝날 때) 일어납니다.

```bash
bash simulation/docker_run.sh python3 -m psi_vtol.sim.run_case --event pusher_off@cruise2+1
```

| 이름 | 일어나는 일 |
|---|---|
| `abort` | 임무 중단. 탑재 failsafe처럼 부드럽게 멈춘 뒤 PX4 Hold로 넘김 |
| `silence` | setpoint가 끊김(Jetson이 죽은 경우). PX4의 offboard 상실 동작이 처리 |
| `transition` | 고정익 천이 명령(버튼이나 스위치를 잘못 누른 경우) |
| `pusher_off` | 푸셔 추력이 0이 됨. 공력 플러그인이 끄고, PX4에는 알리지 않음 |
| `battery` | 배터리가 몇 초 만에 위험 수준(7%)까지 떨어짐. 비상 수준(5%)에는 닿지 않음. PX4의 배터리 failsafe가 처리 |

PX4 v1.17의 고장 주입은 ESC 텔레메트리만 지우고 모터는 그대로 돌기 때문에, 푸셔는 플러그인에서 끕니다
(`/model/psi_qp_<instance>/rotor_off` 토픽에 joint 이름). GPS 상실은 Gazebo에서 주입할 수 없습니다.

## 2. 한 번의 실행 과정

1. 기체·PX4·임무 YAML을 읽어 케이스 값을 정합니다.
2. 기체 SDF, 바람을 넣은 world, Gazebo 서버 설정, 실행 manifest를 케이스 폴더에 씁니다.
3. headless Gazebo와 PX4(airframe 4004)를 띄우고, 부팅 후 파라미터를 하나씩 설정해 응답값으로
   확인합니다.
4. MAVLink runner가 `Profile`의 setpoint를 보내 임무를 비행합니다. 시뮬레이션 시간은 실제의 4배로
   흐릅니다.
5. PX4 ULog와 플러그인이 쓴 `aero.csv`를 읽어 지표를 계산하고 합격 여부를 정합니다.

프로파일이 경로를 따라가지 못함을 알아채거나, 단계가 시간 상한을 넘기거나, 임무가 중단되면 runner는 탑재
노드와 같은 일을 합니다. 기체를 부드럽게 멈추고(`profile.Stop`) PX4 Hold에 넘기며, 푸셔 고장이면 Land에
넘깁니다. 그 뒤 기체가 내려올 때까지 따라갑니다. Hold는 15초 지켜본 뒤 조종자가 하듯 Land를 명령하고,
Return이나 Land는 PX4에 맡깁니다. PX4가 스스로 offboard에서 빠졌을 때도 같습니다. 이런 실행은 임무를
마치지 못했으므로 실패지만, 따로 `safe`(전복 없이, tilt 30° 이하로, 1 m/s 이하로 착지)를 기록합니다.

다음 중 하나가 일어나면 즉시 중단하고 실패로 기록합니다.

- 단계 시간 상한 초과([비행 순서](02_control_concept.md#1-비행-순서))
- roll 또는 pitch 60° 초과
- 상승·착륙이 아닌 단계에서 disarm, offboard 이탈, 지면 1 m 이하
- PX4 위치·자세 텔레메트리가 2초 넘게 끊김
- 실제 시간 420초 초과

ROS 2 노드도 같은 `Profile`로 비행하지만 통신과 watchdog 경로가 다릅니다([7절](#7-ros-2-노드로-비행하기)).

좌표계는 PX4가 NED/FRD, Gazebo가 ENU/FLU입니다. `vehicle/psi_qp.yaml`의 치수는 x 앞, y 왼쪽, z
위이므로 PX4 모터 위치로 옮길 때 y와 z의 부호를 바꿉니다.

## 3. 물리 모델

수식은 Gazebo와 무관한 [`AeroPhysics.hh`](../simulation/gazebo/AeroPhysics.hh)에, Gazebo 연결부는
[`AeroSystem.cc`](../simulation/gazebo/AeroSystem.cc)에 있습니다.

### 강체와 센서

6자유도 강체에 질량, 대각 관성, CG 편차, 중력, 지면 충돌이 들어갑니다. 모터 자리마다 5 g짜리 보조
링크가 있어서 SDF의 총질량은 설정값보다 25 g 큽니다. IMU, 자력계, 기압계, GPS는 PX4 기본 VTOL
모델의 잡음 설정을 그대로 쓰고, 상태 추정은 PX4가 합니다.

### 날개와 평판

주익은 좌우 두 면, 뒤쪽 평판은 한 면으로 계산합니다. 각 면이 느끼는 공기 속도는
`v_local = v_body − wind_body + ω × r`이어서, 기체가 회전하면 양쪽 날개가 서로 다른 속도를 느낍니다.

받음각이 작을 때는 다음 식을 씁니다.

```text
q = 0.5 ρ V²
CL = CLα (α + α0)
CD = CD0 + CL² / (π e AR)
L = q S CL,  D = q S CD,  M = q S c Cm0
```

- 주익의 양력 기울기는 2D 값(`wing_cla_2d`)에 유한 날개 보정을 한 값입니다.
- `wing_alpha0_deg`에는 zero-lift 각을 부호를 바꿔 저장합니다. NACA 4415의 zero-lift 각이 약
  −4°라서 4.0입니다.
- 공력 중심 위치(`wing_cp_x_m`)와 날개 고유의 pitching moment(`wing_cm0`)는 따로 들어갑니다.
- 받음각이 실속각(기본 14°)을 넘으면 blend 폭(기본 8°)에 걸쳐 평판 근사로 부드럽게 넘어갑니다. 평판
  영역은 `CL = 1.8 sinα cosα`, `CD = CD0 + 1.8 sin²α`이고 Cm0는 0으로 줄어듭니다. 그래서 역류나 수직
  유동에서도 힘이 끊기지 않습니다.
- 평판의 유도항력 계수 0.12와 실속각 12°는 가정값입니다.

### 동체와 로터 항력

동체는 축마다 `Fi = −0.5 ρ (CD·A)i vi |vi|`의 항력을 받고, 작용점에 따른 모멘트가 더해집니다. 작용점의
x 위치는 `side_cp_x_m`이고 y·z는 0입니다. 기본값은 CG와 같습니다. 모든 축에 작은 선형 각속도
감쇠(`angular_damping_nm_per_radps`)가 있습니다.

로터는 회전축에 수직인 공기 흐름에 대해 `−kdrag × (정규화 회전수) × V⊥`의 항력을 받습니다. 계수는
`lift_drag_at_full_rpm`, `pusher_drag_at_full_rpm`이고, Gazebo motor model 자체의 로터 항력은 0으로
꺼서 두 번 계산되지 않게 했습니다.

### 추진

Gazebo의 기본 motor model이 1차 회전수 응답, 정적 추력 `T ∝ ω²`, 반토크를 계산하고, 플러그인은 그
추력에서 손실분만 더합니다.

```text
f = 정규화 회전수
g = clamp(1 − max(0, V축방향) / (Vzero × f), 0, 1) × (전압 비율)² × 장착 효율
T = T정적 × g
```

- 축방향으로 공기가 들어오면(전진하는 푸셔, 상승하는 리프트 로터) 추력이 줄어듭니다. 반대 방향
  유입으로 추력이 늘어나는 효과는 넣지 않았습니다.
- 앞뒤 리프트 로터의 장착 효율을 따로 줄 수 있습니다(`front_rotor_efficiency`,
  `rear_rotor_efficiency`). 속도나 자세에 따라 변하지 않는 상수입니다.
- 전압 비율은 Gazebo 시간 0초부터 600초에 걸쳐 1에서 0.9로 내려가는 스크립트입니다. 소비 전력으로
  계산하는 배터리 모델이 아니고, 이륙 전 대기 시간에도 내려갑니다.
- 1500 / 3500 rad/s는 PX4와 Gazebo 사이의 명령 스케일이며 실제 RPM이 아닙니다.

### 바람

평균 수평풍과 수직풍, 수평 풍속·풍향의 느린 사인 변동과 잡음, 수직 잡음을 넣을 수 있습니다.
`wind_from_deg`는 처음 기수 방향을 기준으로 0이 정풍, 90이 왼쪽, 180이 배풍, 270이 오른쪽입니다.
Gazebo WindEffects가 world의 바람을 바꾸고 플러그인이 그 값을 읽습니다. WindEffects가 직접 주는 힘은
0으로 꺼 두었습니다. 공간적으로 균일한 바람이고 잡음 seed를 고정하지 않아서, 같은 케이스도 실행할
때마다 결과가 조금씩 다릅니다.

## 4. 모델에 없는 것

| 빠진 것 | 그래서 판단할 수 없는 것 |
|---|---|
| 측정한 NACA 4415 polar, Reynolds 수 변화, 동적 실속 | 실속 속도, 고받음각 제동 |
| 날개-로터 후류, 평판의 downwash, 간섭 | 실제 전후 로터 효율과 pitch trim |
| 동체·붐의 받음각/옆미끄럼별 힘과 모멘트, 압력 중심 이동 | 동체가 만드는 외란, pitch/yaw 안정성, 측풍 한계 |
| 프로펠러 CT/CQ 지도, windmilling, vortex ring state | 고속·급하강·역류에서의 정확한 추력 |
| 전류, ESC 제한, 발열, 배터리 SOC | 비행 시간, 효율, 열 여유 |
| 프로펠러 자이로 효과, 구조 진동 | 진동과 구조 모드의 영향 |
| 지면 효과, 착륙 다리와 충격 | 착륙 충격과 구조 하중 |
| 푸셔 외의 모터 고장, 센서 고장, 통신 지연 분포 | 그런 고장 시 동작, 실패 확률 |
| 대기속도에 따른 자동 속도 제한, custom 제어 | stock PX4보다 나은 제어의 효과 |
| 목적지 좌표, 거리 기준 제동, 장애물 회피, vision | 목적지 도착 정확도, 정밀 착륙 |

동체와 붐이 받는 하중은 특히 단순합니다. 축별 항력뿐이고 기본 작용점이 CG라서 측풍에 의한 yaw
모멘트가 작게 나옵니다. 측풍 시험 결과를 실기체의 측풍 한계로 쓰려면, 전체 기체의 6축 하중을
받음각·옆미끄럼·속도·RPM별로 측정하거나 CFD로 구해서 이 부분을 바꿔야 합니다.

## 5. 합격 기준

기준값은 [`metrics.LIMITS`](../psi_vtol/sim/metrics.py)에 있습니다. 개발 중에 쓰는 기준이고 비행
허가 기준은 아닙니다. 지표가 하나라도 계산되지 않았거나 NaN이면 실패입니다.

| 지표 | 기준 | 계산 구간 |
|---|---|---|
| roll 또는 pitch 절댓값 | ≤ 30° | 상승 시작부터 착륙 끝까지 |
| 고도 오차 | ≤ 3 m | 첫 호버부터 마지막 경유점 호버 끝까지 |
| 리프트 모터 상한/하한 도달 비율 | 각각 ≤ 5% | 각 구간의 가속–감속과 모서리, 어느 모터든 ≥ 0.98 / ≤ 0.05 |
| 푸셔 포화 비율 | ≤ 10% | 각 구간의 가속–감속과 모서리, 명령 ≥ 0.88 |
| 토크 미달 비율 / 최장 연속 시간 | ≤ 10% / ≤ 1초 | 각 구간의 가속–감속과 모서리 |
| 기수 오차(요구 기수 대비) | ≤ 20° | 각 구간의 가속–감속과 모서리 |
| 계획한 경로(모서리 곡선 포함)에서 벗어난 거리 | ≤ 3 m | 각 구간의 가속–감속, 모서리 |
| 순항 속도 오차(그 구간의 속도 대비) | ≤ 1 m/s | 각 구간 순항의 마지막 5초 |
| 착륙 위치 오차(마지막 경유점 기준) / 속력 | ≤ 2 m / ≤ 1 m/s | land detector가 처음 접지를 알린 시점 / 그 전 3초의 최대 속력 |
| ULog / 공력 로그 최대 공백 | ≤ 1초 / ≤ 0.5초 | ULog는 비행 구간, 공력은 전체 |
| 주익 수직 힘 / 무게 | ≤ 40% | 공력 로그 전체 |

이 밖에 임무 완료, land detector의 착륙 확인, 비행 전체를 덮는 `aero.csv`가 있어야 합니다. 계획에 순항
단계가 하나도 없으면(정사각형을 6 m/s 이상으로 통과 비행하면 모서리 사이에서 가속과 감속만 함) 순항 속도
오차는 보지 않습니다. 대신 기체가 기준점보다 가장 많이 뒤처진 거리(`lag_max_m`)를 기록합니다. 판정에는
쓰지 않고, 8 m를 넘으면 프로파일이 비행 중에 고장으로 처리합니다.

지표를 읽을 때 알아 둘 점:

- 주익 수직 힘은 주익 두 면이 만드는 world 수직 힘의 합입니다. pitch가 있으면 항력의 수직 성분도
  들어가고, 평판과 동체의 힘은 빠집니다.
- 포화 비율은 명령값을 기준으로 하므로 N 단위의 추력 여유와는 다릅니다.
- 착륙 속력은 land detector가 처음 접지(`ground_contact`, 없으면 `maybe_landed`나 `landed`)를 알리기
  전 3초 동안의 최대 EKF 속력입니다. PX4는 기체가 멈춘 뒤에야 접지를 알리므로 그 순간의 속력은 거의
  0입니다. 정상적인 Land라면 하강 속도(`MPC_LAND_SPEED` 0.5 m/s) 근처가 나옵니다.
- 고도 오차는 상승이 끝난 뒤부터 하강 전까지만 봅니다. 상승·하강 중 수직 추종은 단계 도달 여부, 시간
  상한, 전 구간 tilt로만 감시합니다.

## 6. 스윕

```bash
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep oat
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep wind
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep corners
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep random --n 100 --seed 1
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep scenarios
bash simulation/docker_run.sh python3 -m psi_vtol.plots
```

케이스는 기본으로 한 번에 하나씩 비행합니다. Gazebo는 PX4의 모터 명령을 기다리지 않으므로, 시뮬레이션을
여러 개 동시에 돌려 PX4가 뒤처지면 실제 기체에 없는 제어 지연이 생깁니다(4 CPU 컴퓨터에서 함께 돌린
정사각형에 1 Hz 흔들림이 생겼습니다). `--jobs`를 늘리려면 CPU가 충분한지 먼저 확인하세요. 결과의
`flight.other_simulators`에 함께 돌던 PX4 수가 남습니다.

| 스터디 | 내용 | 케이스 수 |
|---|---|---|
| `oat` | 불확실한 값을 하나씩 범위의 양 끝으로 | 61 (기준 기체 포함) |
| `wind` | 2/4/6/8 m/s × 8방향 × 기수 유지 정책 2가지 | 64 |
| `corners` | 여러 불리한 조건을 일부러 겹친 시험 | 7 |
| `random` | 모든 값을 범위 안에서 균등하게 무작위로 | `--n` |
| `scenarios` | 순항 3–7 m/s × 통과 비행/멈춰서 돌기, 돌풍 섞인 6 m/s 바람, 위 다섯 가지 사건 | 30 |

- 범위는 `study.yaml`의 `sweep`에 있습니다. random은 이 순서대로 값을 뽑으므로 순서를 바꾸면 같은
  seed라도 다른 기체가 나옵니다.
- wind의 soft 정책은 yaw 게인을 낮춰 기수가 바람에 밀리게 둔 설정입니다. 기수 오차 기준 20°는
  그대로라서 실패할 수 있습니다.
- corner는 실패 조건을 찾으려는 시험이므로, 통과하도록 수치를 맞추지 않습니다.
- random은 정한 범위에서 고르게 뽑은 표본이라 통과율을 실제 성공 확률로 읽을 수 없습니다.
- scenarios의 사건 케이스는 임무 합격이 아니라 결과로 판정합니다. 안전하게 내려왔는지(`safe`)와, 그
  사건에 대해 기대한 대응이 있었는지(`sweep.EXPECTED`, 예: 푸셔 고장이면 고장을 알아채고 Land)를 봅니다.

### 결과 파일

케이스마다 `out/sitl-runs/<case>/run-<id>/`에 남습니다(컨테이너 안에서는
`/workspace/out/sitl-runs/...`). Docker 없이 돌리면 `PSI_RUN_ROOT`, 기본값 `/tmp/psi_qp_runs`입니다.

| 파일 | 내용 |
|---|---|
| `manifest.json` | 케이스 값, 요청한 PX4 파라미터, 합격 기준, Docker 이미지 ID, OS, repo·PX4 commit과 dirty 여부, 소스·플러그인 hash, 패키지 버전 |
| `models/`, `worlds/`, `server.config` | 생성된 Gazebo 입력 |
| `runtime_env.json` | Gazebo partition 등 실행 환경 |
| `px4.log`, `log/.../*.ulg` | PX4 콘솔 출력, ULog |
| `aero.csv` | 약 20 Hz로 기록한 Gazebo 시간, 대기속도, 주익 수직 힘과 비율, 전압 비율, 로터별 추력 |
| `result.json` | 단계 시간, 실효 파라미터 덤프, 지표, 실패 이유. 사건이 있었으면 그 시각, failsafe 이유와 넘긴 모드, PX4가 거친 모드, `safe` |

- 전체 파라미터 덤프는 30초 안에 다 받지 못하면 `complete: false`로 남습니다. 요청한 파라미터는 이와
  별개로 하나씩 응답값을 확인합니다.
- `aero.csv`는 Gazebo 시간, 단계 시간은 PX4 부팅 시간 기준입니다. 두 시계를 같다고 보고 행을 맞추면 안
  됩니다.
- custom PX4 checkout의 untracked 파일은 manifest에 남지 않으니, 비교할 펌웨어는 commit해 둡니다.

스윕 표는 `out/<mission>/aeroprop-v2/<study>.csv`와 `.json`에, 그림은
`missions/<mission>/figures/aeroprop-v2/`에 저장됩니다. runner가 실패한 케이스는 이전 실행의 결과
파일이 있어도 통과로 치지 않습니다.

## 7. ROS 2 노드로 비행하기

기본 Docker 이미지에는 ROS 2가 없습니다. ROS 2, Micro XRCE-DDS Agent, PX4가 서로 통신할 수 있는
환경을 따로 준비해야 합니다([4. 탑재 소프트웨어](04_onboard.md)). `docker_run.sh`로 컨테이너를 세 개
띄우면 서로 연결되지 않습니다.

터미널 세 개에서 각각 실행합니다.

```bash
python3 -m psi_vtol.sim.start
```

```bash
MicroXRCEAgent udp4 -p 8888
```

```bash
ros2 launch vtol_mission mission.launch.py auto_arm:=true
ros2 topic pub --once /mission/start std_msgs/msg/String "{data: START}"
```

`PX4 is up` 메시지가 나온 뒤 START를 보냅니다. 이 경로는 실시간(배속 1)으로 돌고 실효 파라미터를
`effective_params.json`에 남기지만, 자동 합격 판정은 하지 않습니다.
