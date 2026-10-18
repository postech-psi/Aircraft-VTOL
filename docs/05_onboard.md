# 5. 탑재 소프트웨어 (ROS 2)

Companion computer에서 도는 미션 노드입니다. 패키지는 `ros2/ws/src/vtol_mission` 하나입니다.

## 1. 구성

노드는 네 개입니다. 전체 흐름은 [0. 전체 구조](00_architecture.md#3-탑재-비행-소프트웨어)에 그림으로 있습니다.

| 노드 | 파일 | 하는 일 |
|---|---|---|
| `mission_manager` | `mission_manager.py` | 임무를 단계 목록 순서대로 진행합니다. `WAIT_START → PATH → LANDING → MISSION_DONE`, 문제가 생기면 어디서든 `FAILSAFE` |
| `setpoint_mux` | `setpoint_mux.py` | 지금 단계의 setpoint만 PX4로 보냅니다. PX4에 setpoint를 보내는 노드는 이것 하나입니다 |
| `path_node` | `path_node.py` | 단계 `PATH`: 상승, 호버, 경로 비행. 경로는 통과 비행(기본)이나 멈춰서 돌기. `auto_arm:=true`이면 스스로 offboard로 바꾸고 arm하고, 아니면 조종자가 합니다. 천이 명령은 보내지 않습니다 |
| `landing_node` | `landing_node.py` | 단계 `LANDING`: 마지막 경유점(기본은 이륙 지점) 위에 자리를 잡고 기수를 바람 쪽으로 돌린 뒤 하강하다가, 4 m 아래에서 PX4 Land 모드에 넘깁니다 |

| 토픽 | 보내는 쪽 | 받는 쪽 | 내용 |
|---|---|---|---|
| `/mission/start`, `/mission/abort` | 운용자 | mission_manager | 임무 시작, 중단 |
| `/mission/state` | mission_manager | 단계 노드 | 지금 단계의 이름. 같은 이름의 노드만 활성 |
| `/mission/status` | mission_manager | 운용자 | 임무 상태 |
| `/setpoint_mux/source` | mission_manager | setpoint_mux | 통과시킬 단계 |
| `/path/trajectory_setpoint`, `/landing/trajectory_setpoint` | 단계 노드 | setpoint_mux | 그 단계의 setpoint |
| `/path/status`, `/landing/status` | 단계 노드 | mission_manager | `ACTIVE:…`, `DONE`, `FAILED:<이유>` |
| `/setpoint_mux/status` | setpoint_mux | 운용자 | 지금 무엇을 보내는지 |
| `/fmu/in/trajectory_setpoint`, `/fmu/in/offboard_control_mode` | setpoint_mux | PX4 | PX4가 따를 setpoint |
| `/fmu/in/vehicle_command` | path_node, landing_node, mission_manager | PX4 | offboard와 arm(path_node, `auto_arm`일 때), Land(landing_node), FAILSAFE의 Hold나 Land(mission_manager) |
| `/fmu/out/vehicle_local_position_v1` | PX4 | 단계 노드, setpoint_mux | 위치, 속도, 기수 |
| `/fmu/out/vehicle_attitude` | PX4 | 단계 노드 | 자세(회전이 끝났는지, 착륙 전 바람 방향) |
| `/fmu/out/vehicle_status_v1` | PX4 | 단계 노드, mission_manager | arm 여부, 비행 모드 |

PX4 쪽 토픽 이름은 `px4.py` 한 곳에 있고 노드 파라미터 `px4_topic.*`로 바꿀 수 있습니다. PX4 v1.17.0과
px4_msgs v1.17.0에 맞춰져 있습니다([버전](03_px4_setup.md#1-버전)).

`path_node`와 `landing_node`의 setpoint는 `psi_vtol/profile.py`가 만듭니다. 시뮬레이션이 비행한 것과
같은 코드이고, 고도·속도·가속 한계·경로는
[`missions/2026_year_end/mission.yaml`](../missions/2026_year_end/mission.yaml)에서 읽습니다.

### 단계 노드의 규칙

단계 하나가 노드 하나이고, 모두 `source.py`의 `SetpointSource`를 상속해 같은 규칙을 따릅니다.

| 방향 | 토픽 | 내용 |
|---|---|---|
| 받음 | `/mission/state` | 값이 자기 단계 이름과 같을 때만 동작 |
| 보냄 | `/<단계>/trajectory_setpoint` | setpoint. `setpoint_mux`가 PX4로 전달 |
| 보냄 | `/<단계>/status` | `ACTIVE:<상태>`, `DONE`, `FAILED:<이유>` |

새 단계를 넣으려면 `SetpointSource`를 상속한 노드를 만들고 `launch/mission.launch.py`의 `STEPS`에 이름을
추가합니다. `mission_manager`와 `setpoint_mux`는 고칠 필요가 없습니다. vision 정밀 착륙도 이렇게
`LANDING` 자리에 들어갈 예정입니다([7. 할 일](07_roadmap.md#5-vision-vtol로-가는-길)).

### 문제가 생겼을 때

| 상황 | 동작 |
|---|---|
| 단계가 `FAILED`를 보고하거나 `/mission/abort`에 `ABORT` 수신 | mission_manager가 `FAILSAFE`로 가고, mux가 기체를 부드럽게 멈춤(HOLD). 6초 뒤 아직 offboard면 PX4에 넘김: 앞으로 갈 수 없는 고장(`psi_vtol.profile.needs_land`: 푸셔 고장, 크게 뒤처짐)이면 Land, 아니면 Hold |
| 앞으로 나가지 못함(가속 3초 뒤 0.5 m/s 미만) 또는 계획보다 1.5 m/s 넘게 느린 채 속도를 얻지 못함 | PATH가 `FAILED:pusher: no forward speed ...` 또는 `FAILED:pusher: not speeding up ...`. 위처럼 멈춘 뒤 그 자리에 착륙 |
| 기준점보다 8 m 넘게 뒤처짐 | PATH가 `FAILED:falling behind the route ...`. 앞으로 갈 수 없다는 뜻이므로 역시 멈춘 뒤 착륙 |
| 단계 노드의 status가 2초 동안 오지 않음 | `FAILSAFE` |
| 조종사나 PX4 failsafe가 offboard에서 빼냄 | 노드가 `FAILED:left offboard in <단계>`를 보고. 그 뒤는 조종사와 PX4가 맡음 |
| 단계가 시간 상한을 넘김 | `FAILED:timeout in <단계>` |
| 단계 노드가 받는 PX4 위치·상태가 1초 넘게 끊기거나 유효하지 않음 | `FAILED:PX4 state telemetry stale`. 다른 단계가 선택될 때까지 유지 |
| 선택된 단계의 setpoint가 0.5초 넘게 끊김 | mux가 HOLD(부드럽게 멈춤) |
| mux가 받는 위치가 1초 넘게 끊기거나 유효하지 않음 | mux가 offboard 스트림을 멈춤. 이후 PX4의 `COM_OF_LOSS_T`와 loss 동작이 적용됨 |

HOLD가 기체를 멈추는 방법과 Hold·Land를 고르는 이유는 [2. 비행 방법](02_flight.md#5-고장-대응)에 있습니다.
mission_manager는 PX4가 아직 offboard일 때만 기체를 넘기고, 조종사나 PX4 failsafe(지오펜스, 배터리, offboard
상실)가 이미 가져갔으면 손대지 않습니다. Jetson이나 링크가 끊겨 mux의 스트림 자체가 멈추면 PX4가 1초 뒤 Hold에
들어갑니다([3. PX4 설정](03_px4_setup.md#4-파라미터)).

## 2. 빌드

px4_msgs는 저장소에 넣지 않고 태그를 고정해서 받습니다.

```bash
git clone --depth 1 -b v1.17.0 https://github.com/PX4/px4_msgs.git ros2/ws/src/px4_msgs
```

노드는 `psi_vtol` 패키지를 repo에서 바로 읽으므로 repo 최상위를 `PYTHONPATH`에 넣어야 합니다. repo
최상위에서 실행합니다.

```bash
export PYTHONPATH=$PWD:$PYTHONPATH
cd ros2/ws
colcon build
source install/setup.bash
```

## 3. 실행

```bash
ros2 launch vtol_mission mission.launch.py
ros2 topic pub --once /mission/start std_msgs/msg/String "{data: START}"
```

- 기본값에서는 노드가 arm하지 않습니다. START 뒤 조종사가 arm하고 offboard로 바꾸면 상승을 시작합니다.
- 시뮬레이션에서는 `auto_arm:=true`를 붙여 노드가 offboard 전환과 arm을 요청하게 합니다.
- 다른 임무를 날리려면 `mission:=<폴더 이름>`을 붙입니다.
- 진행 상황은 `/mission/status`, `/path/status`, `/landing/status`, `/setpoint_mux/status`를
  `ros2 topic echo`로 봅니다.

## 4. Jetson

`deploy/jetson/`에 Docker 구성이 있습니다. Pixhawk와 시리얼로 연결된 uXRCE-DDS agent 컨테이너와 ROS 2
Humble 컨테이너 두 개이고, repo 전체가 `/workspace`에 마운트됩니다.

```bash
docker compose -f deploy/jetson/compose.yaml up -d --build
docker exec -it ros2 bash
```

컨테이너 안에서 2절의 px4_msgs clone과 `colcon build`를 한 번 하고, 3절대로 실행합니다. 시리얼 장치와
속도는 `compose.yaml`의 `PX4_UXRCE_DEV`, `PX4_UXRCE_BAUD`로 바꿉니다.

이 구성은 PSI-vtol-team의 Jetson 구성에서 카메라와 vision 패키지를 뺀 것입니다. 새 구성으로 Jetson에서
빌드해 본 적은 아직 없습니다.
