# PX4

stock PX4 v1.17.0을 씁니다. 이 폴더에는 버전 정보와 파라미터만 있고 펌웨어 수정은 없습니다. Gazebo의
공력 플러그인은 시뮬레이터 쪽 코드라서 펌웨어와 관계없습니다. custom PX4를 만든다면 어떤 순서로 할지는
[6. 연구 배경과 custom PX4](../docs/06_research_and_custom_px4.md)에 정리했습니다.

## 버전

| | 버전 |
|---|---|
| PX4-Autopilot (Pixhawk, SITL) | v1.17.0 |
| px4_msgs (ROS 2) | v1.17.0 (`86d8239`) |
| Micro-XRCE-DDS-Agent | v2.4.3 (`deploy/microxrce-agent/Dockerfile`) |

Pixhawk, 시뮬레이션, ROS 2 메시지의 버전을 맞춥니다. 버전이 다르면 토픽 이름이나 메시지 정의가 어긋나서
노드가 PX4 데이터를 받지 못합니다.

```bash
git -C ~/px4-v1.17.0 describe --tags
```

## ROS 2 토픽 이름

PX4는 메시지 정의의 `MESSAGE_VERSION`이 0보다 크면 토픽 이름 끝에 `_v<N>`을 붙입니다. v1.17.0에서 노드가
쓰는 토픽은 다음과 같습니다(v1.17.0 소스와 SITL에서 확인).

| 토픽 | 방향 |
|---|---|
| `/fmu/out/vehicle_local_position_v1` | PX4 → 노드 |
| `/fmu/out/vehicle_status_v1` | PX4 → 노드 |
| `/fmu/out/airspeed_validated_v1` | PX4 → 노드 (아직 쓰지 않음) |
| `/fmu/in/trajectory_setpoint` | 노드 → PX4 |
| `/fmu/in/offboard_control_mode` | 노드 → PX4 |
| `/fmu/in/vehicle_command` | 노드 → PX4 |

코드에서는 `ros2/ws/src/vtol_mission/vtol_mission/px4.py`에만 적혀 있습니다. PX4 버전을 올릴 때는 이 표,
그 파일, px4_msgs 태그를 함께 바꿉니다. 실제 이름은 이렇게 확인합니다.

```bash
ros2 topic list | grep /fmu/
```

## 파라미터

[`params.yaml`](params.yaml)에 세 묶음으로 나뉘어 있습니다. 값마다의 이유는
[2. 제어 개념과 PX4 설정](../docs/02_control_concept.md#3-px4-설정)에 있습니다.

| 묶음 | 내용 | Pixhawk에 |
|---|---|---|
| `concept` | 제어 개념. 실기체와 시뮬레이션에서 같음 | 올림 |
| `per_vehicle` | 기체마다 다른 값. 지금은 시뮬레이션 기체의 값 | 측정값으로 바꿔서 올림 |
| `sim_only` | RC와 데이터 링크 failsafe를 끄는 시뮬레이션용 설정 | **올리지 않음** |

모터 위치(`CA_ROTORn_PX`, `_PY`)는 이 파일에 없고 `vehicle/psi_qp.yaml`의 치수에서 계산합니다.

모터 기하와 `concept`, `per_vehicle`을 QGC 형식의 후보 파일로 뽑을 수 있습니다.

```bash
python3 -m psi_vtol.px4_setup --output /tmp/psi_candidate.params
```

측정 전의 가정값이고 출력 핀, 센서 보정, failsafe는 들어 있지 않습니다. SITL airframe 4004는 Gazebo
전용이므로 실기체의 airframe으로 쓰지 않습니다.
