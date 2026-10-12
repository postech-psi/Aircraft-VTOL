# PX4

펌웨어는 고치지 않고 stock 그대로 씁니다. 이 폴더에는 버전과 파라미터만 있습니다.

## 버전

| | 버전 |
|---|---|
| PX4-Autopilot (Pixhawk, SITL) | **v1.17.0** |
| px4_msgs (ROS 2) | **v1.17.0** (`86d8239`) |
| Micro-XRCE-DDS-Agent | v2.4.3 (`deploy/microxrce-agent/Dockerfile`) |

Pixhawk, 시뮬레이션, ROS 2 메시지를 같은 버전으로 맞춥니다. 버전이 다르면 토픽 이름이나 메시지
정의가 어긋나서 노드가 PX4의 데이터를 받지 못합니다.

```bash
git -C ~/px4-v1.17.0 describe --tags
```

## ROS 2 토픽 이름

PX4는 메시지 정의의 `MESSAGE_VERSION`이 0보다 크면 토픽 이름 끝에 `_v<N>`을 붙입니다. v1.17.0에서
노드가 쓰는 토픽은 다음과 같습니다(v1.17.0 소스와 SITL에서 확인).

| 토픽 | 방향 |
|---|---|
| `/fmu/out/vehicle_local_position_v1` | PX4 → 노드 |
| `/fmu/out/vehicle_status_v1` | PX4 → 노드 |
| `/fmu/out/airspeed_validated_v1` | PX4 → 노드 (아직 쓰지 않음) |
| `/fmu/in/trajectory_setpoint` | 노드 → PX4 |
| `/fmu/in/offboard_control_mode` | 노드 → PX4 |
| `/fmu/in/vehicle_command` | 노드 → PX4 |

코드에서는 `ros2/ws/src/vtol_mission/vtol_mission/px4.py` 한 곳에만 적혀 있습니다. PX4 버전을
올릴 때는 이 표와 그 파일, px4_msgs 태그를 함께 바꿉니다. 실제 이름은 이렇게 확인합니다.

```bash
ros2 topic list | grep /fmu/
```

## 파라미터

[`params.yaml`](params.yaml)에 세 묶음으로 있습니다. 이유는
[docs/02_control_concept.md](../docs/02_control_concept.md)의 "PX4 설정"에 있습니다.

| 묶음 | 내용 | Pixhawk에 |
|---|---|---|
| `concept` | 제어 개념. 기체와 시뮬레이션에서 같음 | 올림 |
| `per_vehicle` | 기체마다 다른 값. 지금 값은 시뮬레이션 기체의 것 | 측정값으로 바꿔서 올림 |
| `sim_only` | RC와 데이터 링크 failsafe를 끄는 시뮬레이션용 설정 | **올리지 않음** |

모터 위치(`CA_ROTORn_PX`, `_PY`)는 `vehicle/psi_qp.yaml`의 치수에서 계산합니다.
