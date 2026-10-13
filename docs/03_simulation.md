# 3. 시뮬레이션이 하는 일과 실행 방법

현재 모델 이름은 **`aeroprop-v2`**입니다. 실제 PX4 v1.17.0 SITL과 Gazebo Harmonic의
강체 동역학을 연결하고, 이 repo의 C++ 플러그인이 공력과 추진 손실을 계산합니다.
목적은 가정한 기체에서 제어가 어느 조건에서 여유를 잃는지 찾는 것입니다.
실제 기체와의 모델 일치도는 아직 검증하지 않았습니다.

2026-10-13 Docker/Linux ARM64에서 PX4 v1.17.0과 Gazebo 8.15.0을 빌드·실행했습니다.
공력 플러그인 빌드·로딩, 단위/C++ 시험 **53개 통과**, 수정 후 nominal 임무의 전체 비행·착륙과
판정 통과를 확인했습니다. [실행 보고서](../missions/2026_year_end/revised_simulation_report.md)에
실제 수치, 첫 실패의 원인과 추가 stress 시험 결과를 보존합니다.
[`기존 보고서`](../missions/2026_year_end/report.md)의 그림·통과율은 이전 모델 기록입니다.

## 1. 한 번 실행하면 일어나는 일

1. `vehicle/psi_qp.yaml`, `px4/params.yaml`, 임무의 `mission.yaml`/`study.yaml`을 읽습니다.
2. 기체 SDF, 바람이 있는 world, 센서/물리 설정과 실행 manifest를 생성합니다.
3. PX4 airframe 4004를 시작하고 부팅 후 파라미터 응답값을 확인합니다.
4. MAVLink runner가 `Profile`의 위치·속도 setpoint를 보내 전체 임무를 수행합니다.
   ROS 2 비행도 같은 `Profile`을 사용하지만 통신·watchdog 경로는 별도입니다.
5. PX4 ULog와 공력 플러그인의 `aero.csv`를 읽어 판정합니다. 공력 파일이 없으면 실패입니다.

PX4는 NED/FRD, Gazebo 모델은 ENU/FLU 좌표를 사용합니다. 기체 치수 원본은 x 앞, y 왼쪽,
z 위이며 PX4의 모터 위치로 바꿀 때 y/z 부호를 바꿉니다. 힘을 적용할 위치와 관성 CG 편차를
따로 둡니다. CG 스윕에서는 실제 모델만 움직이고 할당기는 nominal 기하를 계속 사용합니다.
할당기의 공통 CT=6.5는 PX4의 작은 행 제거 문턱을 피하기 위한 상대 배율입니다.
|KM|=0.03과 실제 모델의 추력·반토크는 그대로입니다. [설정 이유](02_control_concept.md)를 확인합니다.

## 2. Docker 환경과 첫 실행

**시뮬레이션은 Docker 안에서 실행합니다.** 호스트에는 Docker engine 또는 Docker Desktop만
준비합니다. 이미지 안에는 Ubuntu 24.04, Gazebo Harmonic 8, PX4 v1.17.0 빌드와 Python
의존성을 넣습니다. 호스트에 별도 PX4/Gazebo를 설치하지 않습니다.
상세한 환경 명령과 custom checkout 연결은 [simulation/README.md](../simulation/README.md)에 있습니다.

repo 최상위에서 다음을 실행합니다.

```bash
docker build -f simulation/Dockerfile -t psi-vtol-sitl:v1.17.0-harmonic simulation
bash simulation/docker_run.sh cmake -S simulation/gazebo -B simulation/gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
bash simulation/docker_run.sh cmake --build simulation/gazebo/build -j2
bash simulation/docker_run.sh python3 -m pytest -q
bash simulation/docker_run.sh
```

wrapper는 repo를 `/workspace`에 마운트하고 매번 새 컨테이너를 만듭니다. 이미지의 PX4
위치는 `/opt/px4`, 결과는 호스트에 남는 `/workspace/out/sitl-runs`입니다. 마지막 명령은
nominal 임무입니다. 공력 플러그인이 없으면 runner가 시작하지 않습니다.
Gazebo API/ABI는 major 8이며 [공식 system-plugin API](https://gazebosim.org/api/sim/8/createsystemplugins.html)를 사용합니다.

최초 이미지 빌드는 의존성 다운로드와 PX4 컴파일을 포함합니다. source 변경은 mount로
반영되고 C++ 변경 후에는 플러그인을 다시 빌드합니다. 이미지/패키지를 완전히 lock하지
않았으므로 manifest의 commit·버전·hash를 결과와 함께 보존합니다.
설치 기반은 [Gazebo 안내](https://gazebosim.org/docs/harmonic/install_ubuntu/)와
[PX4 v1.17.0 setup](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/Tools/setup/ubuntu.sh)입니다.

custom PX4는 호스트의 절대 경로를 `PSI_PX4_CHECKOUT`으로 지정해 `/opt/px4`에 마운트하고
같은 이미지에서 `px4_sitl_default`를 빌드합니다. runner는 펌웨어를 자동으로 수정하거나
빌드하지 않습니다. native Ubuntu에서 직접 실행할 때만 `PSI_PX4_ROOT`, `PSI_RUN_ROOT`로
경로를 지정합니다.

## 3. 포함한 물리

구현은 [`AeroPhysics.hh`](../simulation/gazebo/AeroPhysics.hh)의 독립 수식과
[`AeroSystem.cc`](../simulation/gazebo/AeroSystem.cc)의 Gazebo 연결부로 나뉩니다.

### 강체와 센서

기체 본체는 6자유도 강체이며 질량, 대각 관성, CG 편차, 중력과 지면 충돌을 포함합니다.
모터 위치의 5 g 보조 링크 다섯 개도 질량에 더해지므로 총 SDF 질량은 설정 질량보다 25 g 큽니다.
IMU·자력계·기압·GPS 센서는 stock 모델의 잡음 설정을 가져옵니다. 상태 추정은 PX4가 수행합니다.
센서 고장, 시간 동기화 오류, 통신 지연 분포는 별도 모델링하지 않습니다.

### 주익과 뒤쪽 평판

주익을 좌우 두 면으로, 평판을 한 면으로 계산합니다. 각 면 위치의 공기 상대 속도는
`v_local = v_body − wind_body + ω × r`입니다. 회전 때문에 양쪽 날개가 다른 속도를
느끼므로 roll/pitch에 따른 공력 변화 일부가 들어갑니다.

각 면의 시위-수직 평면 속도에서 받음각 α를 구하고 붙임각을 반영합니다.
작은 받음각에서는 다음 근사를 사용합니다.

```text
q = 0.5 ρ Vplane²
CL = CLα (α + α0)
CD = CD0 + CL² / (π e AR)
L = q S CL,  D = q S CD,  M = q S c Cm0
```

주익의 2D 양력 기울기는 유한 날개 보정 후 사용합니다. α0는 zero-lift 각도의 반대 부호를
저장한 값이며 기본 4°입니다. `wing_cp_x_m`는 공력 중심 위치이고 `wing_cm0`는 독립적인
기수 숙임 모멘트입니다. 두 항을 별도로 더해 예전의 "양력 작용점을 뒤로 옮겨 Cm을 흉내 내기"를
없앴습니다. 항력은 양력이 0이어도 남습니다.

기본 주익은 |α|=14°에서 22°까지 smoothstep으로 평판 근사에 섞습니다.
분리 영역에서는 `CL=1.8 sinα cosα`, `CD=CD0+1.8 sin²α`이고 Cm0는 0으로 줄입니다.
그래서 역류·수직 유동에서도 힘이 갑자기 사라지지 않습니다. **정상/역류의 실제 NACA4415
극곡선이나 동적 실속을 재현한 것은 아닙니다.** 평판의 유도항력 계수 0.12와 실속 시작 12°도
가정입니다. span 방향 유동에 의한 주익 측력과 실제 3D sideslip 효과는 빠져 있습니다.

### 동체·로터 항력

동체는 각 축에 `Fi = −0.5 ρ (CD A)i vi |vi|`를 적용합니다. 순수 측풍과 후진에서도 항력이
남고, 지정한 작용점의 모멘트를 더합니다. 작은 선형 각속도 감쇠도 있습니다.
기본 body 작용점은 CG와 같은 위치입니다. 현재 x 위치는 `side_cp_x_m`로 바꾸고 y/z는 0입니다.
따라서 nominal에서는 body drag 자체의 모멘트가 작게 평가될 수 있습니다. 이 가정으로
측풍 시험이 통과했다고 실제 동체의 yaw/pitch 안정성을 확인한 것으로 해석하면 안 됩니다.

**동체가 제어 외란이 되는 효과는 일부만 포함했습니다.** 실제 동체·boom의 받음각/옆미끄럼
별 양력·측력·독립 pitch/yaw/roll 모멘트, 압력 중심 이동과 rotor–body 간섭은 없습니다.
`−dω`는 측정된 안정 미계수도 아니며 yaw/pitch가 실제로 안정하다는 근거로 쓰지 않습니다.
주익·평판·로터 하중은 별도로 더하지만 동체가 그 유동을 바꾸는 효과까지 계산하지 않습니다.
비행시험 전에 전체 기체의 6축 하중 vs α/β/속도/RPM을 측정하거나, 필요한 영역의 CFD와
측정으로 힘·모멘트 지도를 만들어 이 단순 모델을 교체해야 합니다.
이는 동체·붐 전체의 실험적 6축 계수 모델을 대신하는 근사입니다.

로터의 횡류 항력은 `−kdrag × normalized_rpm × Vperpendicular`입니다.
예전 Gazebo 기본 상수를 숨겨 쓰던 부분을 기체 YAML의 별도 계수로 드러냈습니다.
1500/3500 rad/s는 PX4↔Gazebo 명령 스케일이며 실기 RPM으로 해석하면 안 됩니다.

### 추진기

stock Gazebo motor model이 1차 회전수 응답, 정적 `T ∝ rpm²`, 반토크를 계산합니다.
추가 플러그인은 이 힘의 **손실분만** 더하며 stock 횡류 항력은 0으로 꺼 중복을 피합니다.

```text
f = normalized_rpm
g = clamp(1 − max(0,Vaxial)/(Vzero × f), 0, 1) × voltage_ratio² × installed_efficiency
Tactual = Tstatic × g
```

f가 0이면 추력도 0입니다. 푸셔가 전진하거나 리프트 로터가 상승할 때 축방향 유입에 따른
추력 감소를 넣습니다. 역방향 유입으로 추력이 늘어나는 효과는 넣지 않았습니다.
전·후방 리프트 로터의 장착 효율을 따로 줄일 수 있고 실제 힘 위치와 푸셔 roll 반토크도
동역학에 들어갑니다. 설치 효율은 상수이며 속도·자세에 따른 후류 해석은 없습니다.

전압 비율은 Gazebo world 시간 0부터 600 simulation초 동안 1→0.9로 줄어드는 기본 스크립트입니다.
모델 로딩·arming 대기 중에도 시간이 흐르므로 실제 비행 시작 전부터 감소할 수 있습니다.
비행 중 소비 전력·SOC로 계산한 배터리가 아닙니다. 전압² 근사는 모터 RPM 응답과 완전히
결합된 전기 모델도 아닙니다. `Vzero`와 전압·설치 효율 수치는 모두 미측정 민감도 변수입니다.
[실제 quadplane 풍동 연구](https://arxiv.org/html/2301.12316v1)와
[UIUC propeller 데이터](https://m-selig.ae.illinois.edu/props/propDB.html)는 앞으로 추력 지도를
만드는 근거이며 이 기체의 계수를 제공한 것은 아닙니다.

### 바람

평균 수평풍·수직풍, 수평 풍속/방향의 느린 주기 변동과 잡음, 수직 잡음을 지원합니다.
`wind_from_deg`는 초기 기수 기준으로 0 정풍, 90 왼쪽, 180 배풍, 270 오른쪽입니다.
Gazebo WindEffects가 world 바람을 바꾸고 플러그인이 같은 바람을 읽습니다. 본체에
WindEffects 힘을 별도로 중복 적용하지 않습니다. 공간적으로 균일한 바람이며 지형·건물·
프로펠러 후류의 난류가 아닙니다. 난류 seed를 고정하는 재현성은 아직 제공하지 않습니다.

## 4. 포함하지 않은 것

| 빠진 항목 | 그 때문에 말할 수 없는 것 |
|---|---|
| 측정된 NACA4415 polar, Reynolds 변화, 동적 실속·유동 이력 | 실속 속도·고받음각 제동을 정량 검증했다고 할 수 없음 |
| 날개-로터 후류, tail downwash, 동적 간섭 | 실제 전후 로터 효율과 pitch trim을 확정할 수 없음 |
| 동체/boom의 CL·CY·Cl/Cm/Cn 지도, α/β 안정 미계수와 압력 중심 이동 | 실제 body 외란·pitch/yaw 안정성, 측풍 허용 범위를 확정할 수 없음 |
| 프로펠러 CT/CQ 지도, windmilling, inflow dynamics, VRS | 고속·급하강·역류의 정확한 추력 예측 불가 |
| 전력·전류·ESC 제한·발열·SOC 배터리 모델 | 항속 시간·효율 개선·열 여유 판정 불가 |
| 프로펠러 자이로/정확한 회전체 관성, 탄성/진동 | 구조 모드·진동·회전체 결합을 검증하지 않음 |
| 지면 효과, 실제 착륙다리·파손 모델 | touchdown metric을 충격하중·구조 안전성으로 해석할 수 없음 |
| 센서/액추에이터 고장과 통신 장애 분포 | 실제 실패 확률·고장 허용성 판정 불가 |
| 대기속도에 따른 자동 envelope 보호, 통합 force/moment custom 제어 | 현재는 stock 기준선의 한계를 관찰하는 모델 |
| 좌표 기반 목적지·거리 기준 제동, 장애물 회피, vision | 예정 목적지 도착이나 정밀 착륙을 검증하지 않음 |

## 5. 통과 판정

기준 원본은 [`metrics.LIMITS`](../psi_vtol/sim/metrics.py)입니다. 아래는 초기 개발 기준이며
비행 인증 기준이나 측정으로 확정한 허용값이 아닙니다. 하나라도 없거나 NaN/Inf이면 실패합니다.

| 지표 | 기준 | 계산 구간 |
|---|---|---|
| roll 또는 pitch 절댓값 | ≤30° | 상승 시작부터 Land 종료까지 |
| 고도 오차 | ≤3 m | 첫 호버부터 두 번째 호버 종료까지 |
| 리프트 상한 / 하한 비율 | 각각 ≤5% | 가속–감속, 어느 모터든 ≥0.98 / ≤0.05 |
| 푸셔 포화 비율 | ≤10% | 가속–감속, 명령 ≥0.88 |
| 토크 미달 비율 / 연속 시간 | ≤10% / ≤1초 | 가속–감속 |
| 요구 기수와의 오차 | ≤20° | 가속–감속 |
| 초기 진행선에서 횡방향 오차 | ≤3 m | 가속–감속 |
| 실제 진행 방향 순항 속도 오차 | ≤1 m/s | 순항 마지막 5초 |
| 착륙 위치 오차 / 속력 | ≤2 m / ≤1 m/s | land detector가 처음 landed라고 한 시점 근처 |
| 주요 ULog / 공력 로그 최대 공백 | ≤1초 / ≤0.5초 | ULog는 비행 구간, 공력은 전체 로그 |
| 주익 수직 힘 / nominal 무게 | ≤40% | 공력 전체 로그 |

완료, land detector의 landed 확인, 전체 비행 길이를 덮는 공력 로그도 필요합니다.
`wing_lift_fraction`은 주익 두 면이 만드는 **world 수직 힘**의 합이며 pitch 시 항력의 수직
성분도 포함합니다. tail·동체 힘은 제외합니다. 40%는 stock 기준선의 여유를 조사하기 위한
보수적인 시작값이고 물리적 안정성 정리에서 나온 경계가 아닙니다.
명령 포화 비율 역시 추력 여유의 간접 지표이며 실제 N 단위 여유와 같지 않습니다.

착륙 속도는 land detector 시점에 가장 가까운 EKF 속도입니다. 감지 지연·필터링 때문에
충돌 직전 속도보다 작을 수 있습니다. 단순 disarm보다 유용하지만 충격 검증은 아닙니다.
현재 altitude 기준은 고도 유지 구간용입니다. 상승·하강의 수직 경로 추종 오차는 별도
정량 지표가 없고 단계 도달/timeout 및 전 구간 tilt로 감시합니다.

## 6. 스윕과 결과 파일

```bash
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep oat --jobs 2
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep wind --jobs 2
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep corners --jobs 2
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep random --n 100 --seed 1 --jobs 2
bash simulation/docker_run.sh python3 -m psi_vtol.plots
```

OAT는 각 불확실성의 양 끝을 하나씩 바꿉니다. wind는 2/4/6/8 m/s, 8방향, 두 yaw 정책의
64가지입니다. soft yaw 정책도 20° 기수 오차 기준을 그대로 적용하므로 실패할 수 있습니다.
corner는 약한 푸셔+정풍, 높은 양력+제동, 저전압+전후 로터 손실, 양쪽 측풍+돌풍,
배풍 역류를 겹칩니다. corner는 실패 조건을 찾는 시험이며 전부 통과하도록 수치를 조절할 대상이 아닙니다.
random은 선택한 범위의 독립 균등 표본으로 실제 오차 확률분포가 아닙니다.

각 실행은 Docker의 `/workspace/out/sitl-runs/<case>/run-<id>/`에 보존됩니다.
호스트에서는 repo의 `out/sitl-runs/`입니다. native runner 기본값은 `/tmp/psi_qp_runs`입니다.

| 파일 | 내용 |
|---|---|
| `manifest.json` | resolved 수치·요청 파라미터·판정값·Docker image ID·OS/architecture·repo/PX4 commit·dirty 여부·PX4 tracked diff hash·소스/플러그인 hash·패키지 버전 |
| `models/`, `worlds/`, `server.config` | 생성된 실제 입력 |
| `runtime_env.json` | 실제 Gazebo partition과 선택한 simulator 실행 환경; 컨테이너 사이에도 UUID로 격리 |
| `px4.log`, `log/.../*.ulg` | 콘솔, PX4 ULog |
| `aero.csv` | 약 20 Hz simulator 시간·대기속도·주익 수직 힘/비율·전압 비율·각 로터 실제 모델 추력 |
| `result.json` | phases, 요청값 적용 확인 후 실효 파라미터 덤프, 지표와 실패 이유 |

전체 파라미터 덤프는 30초 후 partial로 끝날 수 있고 `complete`에 표시합니다. 요청한 개별
설정의 readback은 별도 필수 검사입니다. `aero.csv`는 simulator 시간, phase는 PX4 boot
시간이므로 두 시계가 같다고 가정해 행을 맞추면 안 됩니다. 현재 공력 판정은 전체 로그의
상한과 길이만 사용합니다. 시간 정렬·추력 잔차 비교는 향후 과제입니다.
custom checkout의 untracked 파일 내용은 manifest에 보관하지 않으므로 비교할 펌웨어는
commit하고 이미지·checkout을 함께 보존하는 편이 좋습니다.

스윕 표는 `out/<mission>/aeroprop-v2/`에, 새 그림은
`missions/<mission>/figures/aeroprop-v2/`에 저장합니다. 실행 실패 시 이전 JSON을 성공 결과로
재사용하지 않습니다. 이전 자료를 명시적으로 그리려면 `plots --model-revision legacy`를 씁니다.
새 nominal이 통과한 뒤에도 값의 출처, 실패 case와 민감도를 함께 검토해야 합니다.
그림은 실제 데이터 범위의 자세·속도를 표시하고 motor 축은 0–1을 모두 보여 줍니다.
중단된 비행은 관측 구간의 진단 지표를 남기지만 완료·착륙 기준이 없어 통과할 수 없습니다.

## 7. ROS 2로 확인하기

기본 Docker 이미지는 MAVLink SITL용이며 ROS 2는 포함하지 않습니다. 아래는 별도 ROS 2
통합 환경의 명령입니다. 이를 기본 wrapper의 독립 컨테이너 세 개에서 실행하면 서로
자동 연결되지 않습니다. ROS 2/agent/PX4의 네트워크와 토픽 namespace를 구성한 뒤
[탑재 소프트웨어 문서](04_onboard.md)의 ROS 환경을 빌드해야 합니다. 터미널을 나누어 실행합니다.

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

`PX4 is up` 이후 START를 보냅니다. ROS 실행은 real-time factor 1을 씁니다.
이 경로는 실효 파라미터를 `effective_params.json`에 남기지만 자동 pass/fail은 수행하지 않습니다.
미션 노드 성공 메시지만으로 새 metrics 판정까지 통과했다고 기록하지 않습니다.
노드·토픽 중단 시험과 실제 Jetson 빌드는 여전히 별도 통합 검증이 필요합니다.
