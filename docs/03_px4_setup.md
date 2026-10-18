# 3. PX4 설정

stock PX4 v1.17.0을 쓰고 펌웨어는 수정하지 않습니다. 바꾸는 것은 [`px4/params.yaml`](../px4/params.yaml)의
파라미터와, `vehicle/psi_qp.yaml`에서 계산하는 모터 기하뿐입니다.

## 1. 버전

| | 버전 |
|---|---|
| PX4-Autopilot (Pixhawk, SITL) | v1.17.0 |
| px4_msgs (ROS 2) | v1.17.0 (`86d8239`) |
| Micro-XRCE-DDS-Agent | v2.4.3 (`deploy/microxrce-agent/Dockerfile`) |

Pixhawk, 시뮬레이션, ROS 2 메시지의 버전을 맞춥니다. 다르면 토픽 이름이나 메시지 정의가 어긋나서 노드가 PX4
데이터를 받지 못합니다. PX4는 메시지의 `MESSAGE_VERSION`이 0보다 크면 토픽 이름 끝에 `_v<N>`을 붙입니다
(`/fmu/out/vehicle_local_position_v1`). 노드가 쓰는 이름은 `vtol_mission/px4.py`에만 있으므로, PX4를 올릴 때는
그 파일과 px4_msgs 태그를 함께 바꾸고 `ros2 topic list | grep /fmu/`로 확인합니다.

## 2. 기체 형식

- SITL은 airframe 4004 `gz_standard_vtol` 위에 우리 파라미터를 얹습니다. 4004는 Gazebo 전용이므로 Pixhawk에는
  쓰지 않습니다.
- 실기체는 standard VTOL을 바탕으로 별도 airframe 설정을 만듭니다. 출발점은 `CA_AIRFRAME=2`, `VT_TYPE=2`,
  로터 5개, 조종면 0개입니다.
- 센서 방향과 보정, 출력 프로토콜, RC, 전원 모듈, 위치 센서는 QGroundControl에서 보드에 맞게 설정합니다.

## 3. 모터 배치

PX4 좌표는 x 앞, y 오른쪽, z 아래(FRD)입니다. 위치는 가정한 CG 기준이므로 실제 CG를 재면 다시 계산합니다.

| 모터 index / 출력 기능 | 위치 | PX4 x, y, z (m) | 추력축 AX, AY, AZ | KM |
|---|---|---|---|---|
| 0 / Motor 1 | 앞 오른쪽, CCW | +0.3325, +0.2525, −0.07 | 0, 0, −1 | +0.030 |
| 1 / Motor 2 | 뒤 왼쪽, CCW | −0.3325, −0.2525, −0.07 | 0, 0, −1 | +0.030 |
| 2 / Motor 3 | 앞 왼쪽, CW | +0.3325, −0.2525, −0.07 | 0, 0, −1 | −0.030 |
| 3 / Motor 4 | 뒤 오른쪽, CW | −0.3325, +0.2525, −0.07 | 0, 0, −1 | −0.030 |
| 4 / Motor 5 | 푸셔 | −0.40, 0, 0 | 1, 0, 0 | 0 |

- 회전 방향은 추력이 나가는 쪽에서 내려다본 기준입니다.
- 모터 index는 보드의 커넥터 번호와 다릅니다. QGC Actuators에서 커넥터마다 Motor 1–5 기능을 지정하고,
  프로펠러를 뗀 채 모터 테스트로 위치와 회전 방향을 하나씩 확인합니다.
- 시뮬레이션에서 CG를 바꾸는 스윕은 Gazebo 모델만 바꾸고 PX4에는 위 기하를 그대로 알려 줍니다. PX4가 모르는
  CG 오차를 흉내 내려는 것입니다.

**`CA_ROTORn_CT=6.5`** 는 다섯 모터에 똑같이 준 할당기의 상대 계수이고 N 단위 추력이 아닙니다. PX4 v1.17은 효과
행렬의 한 행이 모두 절댓값 0.05 이하면 그 축을 지웁니다
([`ControlAllocator.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/control_allocator/ControlAllocator.cpp)).
CT=1, |KM|=0.03에서는 SITL에서 yaw 행이 실제로 지워졌습니다. 배율을 6.5로 올리면 yaw 계수가 0.195가 되어 이
문턱을 넘습니다. 할당은 정규화되므로 물리적인 추력이나 반토크는 바뀌지 않습니다.

**KM과 푸셔 오프셋.** KM은 부호를 포함한 반토크/추력 비이고 지금 값은 eCalc에서 계산했습니다. stock
standard-VTOL 할당기는 푸셔의 반토크를 쓰지 않고, 푸셔의 PY/PZ도 0으로 두었습니다. 그래서 푸셔의 반토크와
추력선 오프셋은 리프트 쪽 자세 제어가 외란으로 받아 상쇄합니다. 실제 추력선 위치는 재 두어야 합니다.

## 4. 파라미터

`params.yaml`은 세 묶음입니다.

| 묶음 | 내용 | Pixhawk에 |
|---|---|---|
| `concept` | 제어 개념. 실기체와 시뮬레이션에서 같음 | 올림 |
| `per_vehicle` | 기체마다 다른 값. 지금은 시뮬레이션 기체의 값 | 측정값으로 바꿔서 올림 |
| `sim_only` | RC와 데이터 링크 failsafe를 끄는 시뮬레이션용 설정 | **절대 올리지 않음** |

| 설정 | 값 | 의미 |
|---|---|---|
| `CA_ROTOR_COUNT`, `CA_SV_CS_COUNT` | 5, 0 | 모터 5개, 조종면 없음 |
| `VT_FWD_THRUST_EN` | 6 | `MPC_LAND_ALT2`(5 m) 위에서만 푸셔 보조. auto Land와 하강에서는 끔 |
| `VT_PITCH_MIN`, `VT_LND_PITCH_MIN` | 0°, 0° | 보조가 켜진 동안 전진을 위해 기수를 숙이지 않음 |
| `VT_FWD_THRUST_SC` | 2.75 | 무게/푸셔 최대 추력에서 시작한 값. 푸셔 벤치 시험 후 다시 정함 |
| `CA_R4_SLEW` | 1.0 s | 푸셔 명령이 0→1로 가는 데 최소 1초 |
| `CA_ROTOR4_PY`, `_PZ` | 0, 0 | 푸셔 오프셋을 할당에 넣지 않음 |
| `THR_MDL_FAC` | 1.0 | 시뮬레이터의 추력이 회전수 제곱이라 1. 실기체는 벤치 곡선으로 정함(eCalc로는 약 0.35) |
| `MC_YAWRATE_K` | 3.0 | SITL에서 맞춘 값. 실기체에서 다시 맞춤 |
| `MPC_XY_VEL_MAX` | 16 m/s | 명령 속도가 잘리지 않도록 둔 상한. 운용 속도가 아님 |
| `VT_ARSP_TRANS`, `VT_F_TRANS_DUR`, `VT_TRANS_TIMEOUT`, `VT_F_TRANS_THR` | 30 m/s, 20 s, 0.5 s, 0 | 고정익 천이가 끝나지 못하게 막음([5절](#5-고정익-천이-막기)) |
| `COM_OF_LOSS_T`, `COM_OBL_RC_ACT` | 1 s, 5 (Hold) | Jetson의 setpoint가 1초 끊기면 Hold. 기본 동작(Position 모드)은 RC 조종자가 있어야 하고, SITL에서는 상실을 알고도 5 m/s로 15초 넘게 직진했음 |
| `COM_LOW_BAT_ACT` | 3 | 배터리 critical에서 귀환, emergency에서 착륙. 기본값은 경고만 |
| `GF_ACTION` | 3 (Return) | 지오펜스를 넘으면 귀환 |
| `GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST` | 경로에서 계산 | 가장 먼 경유점 + 30 m, 비행 고도 + 30 m. 기본 정사각형은 101 m, 50 m |
| `RTL_TYPE`, `RTL_RETURN_ALT`, `RTL_DESCEND_ALT` | 0, 비행 고도, 10 m | MC 그대로 직접 귀환. 기본값대로면 60 m까지 올라간 뒤 돌아옴 |
| `MPC_ACC_HOR` | 1.5 m/s² | PX4 자신의 제동(Hold, Return, Land 접근). 기본값 3에서는 5 m/s에서 멈출 때 25°까지 기울었음 |
| `WV_EN`, `WV_YRATE_MAX` | 1, 15°/s | PX4 자신의 모드에서 기수를 바람 쪽으로. Offboard에서는 노드가 기수를 정함 |
| `MPC_LAND_SPEED` | 0.5 m/s | 착륙 마지막 구간 하강 속도. 기본값 0.7 |

- PX4의 SITL 시작 스크립트는 `COM_OF_LOSS_T`를 배속만큼 늘려 4초로 두므로 파라미터 파일에서 되돌립니다.
- 비행마다 다른 값(지오펜스, 귀환 고도)은 경로에서 계산합니다(`gz_model.mission_params`). 시뮬레이션과 검토용
  파라미터 파일이 같은 계산을 씁니다.
- `VT_FWD_THRUST_EN=6`의 고도 판단에는 거리 센서나 home 기준 고도가 쓰입니다. 경계에 hysteresis가 없으므로
  5 m 근처에서 오래 머물지 않습니다
  ([`vtol_type.h`](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/vtol_att_control/vtol_type.h)).

## 5. 고정익 천이 막기

조종면이 없으므로 고정익 모드로 들어가면 자세를 잡을 수단이 없습니다.

- 노드는 `VEHICLE_CMD_DO_VTOL_TRANSITION`을 보내지 않습니다.
- RC에 천이 스위치를 매핑하지 않고, QGC의 천이 버튼도 누르지 않습니다. VTOL 이륙, 천이, 고정익 미션 항목을
  올리지 않습니다.
- Return은 MC 그대로 돌아오는 `RTL_TYPE=0`만 씁니다(1–3은 천이를 넣음).
- 그래도 천이 명령이 들어올 때를 대비해 천이가 끝날 수 없게 막습니다. 천이 중 푸셔를 끄고(`VT_F_TRANS_THR=0`),
  대기속도 기준을 푸셔로 낼 수 없는 30 m/s로, 시간 기준을 20 s로 두고, 그보다 먼저 `VT_TRANS_TIMEOUT=0.5 s`가
  천이를 취소하게 합니다.

천이를 시도하는 동안 PX4는 offboard setpoint를 따르지 않으므로 취소까지의 시간은 짧을수록 좋습니다. SITL에서
천이 명령을 보내 확인했습니다.

| 설정 | 결과 |
|---|---|
| PX4 기본값, 호버 중 | 푸셔가 기체를 5 m/s까지 밀다가 약 6초 뒤 quad-chute가 취소. 그 순간 roll 43°. 정풍이 8 m/s였다면 천이가 끝났을 것 |
| 취소까지 3 s, 5 m/s 순항 중 | 3초 동안 모서리를 돌지 않고 직진하다가 늦은 모서리를 따라잡느라 roll 25° |
| 취소까지 0.5 s(지금) | 곧 취소. PX4가 푸셔 보조를 끄고 일반 쿼드로 임무 전체를 마침(최대 tilt 14°) |

quad-chute 뒤에는 PX4가 "VTOL fixed-wing system failure"를 기록하므로, 착륙한 뒤 재부팅해야 다시 arm할 수
있습니다.

## 6. 실기체에 올리기

```bash
bash simulation/docker_run.sh python3 -m psi_vtol.px4_setup --output /workspace/out/psi_candidate.params
```

QGC 형식 파일에 모터 기하, `concept`, `per_vehicle`, 기본 임무의 지오펜스와 귀환 고도가 들어가고 `sim_only`는
빠집니다. 출력 핀, 센서 보정, RC는 들어 있지 않습니다. 추력·CG·게인이 아직 가정값이므로, 측정한 값으로 YAML을
고친 뒤 다시 만들어 QGC에서 기존 설정과 비교하고 적용합니다.

실기체에서 따로 확인할 것:

- RC가 있을 때와 없을 때의 offboard 상실, RC·데이터 링크·배터리·위치 추정 상실 시 동작.
- `MPC_LAND_ALT1/2`, `MPC_LAND_SPEED`, `COM_DISARM_LAND`와 지면 기준 고도.
- 전류 센서 보정(`BAT1_A_PER_V`)과 배터리 잔량 추정. 부하가 걸리면 전압이 처지므로 PX4는 내부 저항으로
  보정합니다([Battery Estimation Tuning](https://docs.px4.io/v1.17/en/config/battery)).
- 첫 호버 로그의 진동과 자력계. 진동이 크면 센서가 포화되고 위치 추정이 틀어지며
  ([Flight Review](https://docs.px4.io/v1.17/en/log/flight_review)), 전원선 가까운 자력계는 전류에 따라 기수가
  틀어집니다([Compass Power Compensation](https://docs.px4.io/v1.17/en/advanced_config/compass_power_compensation)).
