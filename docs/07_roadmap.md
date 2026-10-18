# 7. 할 일

## 1. 연말 비행까지

1. **동체팀 수치 확정.** CG 위치, 날개의 전후 위치와 붙임각, 평판 붙임각, 푸셔의 모터·프로펠러와 위치,
   비행 상태의 전비중량. 확정된 값은 `vehicle/psi_qp.yaml`에 넣고 표시를 GIVEN으로 바꿉니다.
2. **벤치 측정.** 리프트 모터와 푸셔의 명령–추력–전류 곡선을 전압별로, 푸셔는 대기속도별로도 잽니다
   ([교훈 2](06_lessons.md#기체)). yaw 권한과 모터 장착 각도, 실제 CG를 잽니다. 그 값으로 `THR_MDL_FAC`, 추력
   한계, `VT_FWD_THRUST_SC`를 정하고 OAT·바람·시나리오를 다시 돌립니다.
3. **Pixhawk 설정.** `python3 -m psi_vtol.px4_setup`으로 후보 파일을 만들고, 모터 순서와 회전 방향, 전류 센서를
   확인한 뒤 QGC에서 출력·센서·failsafe를 설정합니다. `sim_only`는 올리지 않습니다
   ([3. PX4 설정](03_px4_setup.md#6-실기체에-올리기)).
4. **ROS 2 통합 시험.** ROS 환경과 DDS 네트워크를 따로 준비해 상태 지연, offboard 상실, HOLD, 착륙까지
   시험합니다. 지금의 watchdog과 모델로는 아직 시험하지 않았고, Jetson에서 빌드해 본 적도 없습니다.
5. **시험장과 서류.** 시험장에 맞춰 경유점과 지오펜스를 정하고, 사람과 재산에서 100 m 넘게 떨어진 곳을
   고릅니다. 조종자 증명과 기체 신고를 무게로 확인합니다([교훈 14](06_lessons.md#비행-시험)).

## 2. 비행 시험 순서

단계마다 하나만 바꾸고, 앞 단계의 기준을 넘긴 뒤에 다음으로 갑니다. 매 비행의 ULog, 전력, 온도, 배터리
잔량, 착륙 여유를 기록합니다.

| 단계 | 확인할 것 | 다음으로 가는 기준 | 문제가 생기면 |
|---|---|---|---|
| 1. 구속 호버 | 모터 순서와 방향, 진동, 자력계, 호버 추력 | 진동과 자력계가 정상([교훈 13](06_lessons.md#비행-시험)) | 모터 정지 |
| 2. 낮은 호버 튜닝 | 각속도 → 고도 → 위치 게인, yaw 권한 | 작은 roll 입력에 두 번 안에 가라앉음, 기수 유지 | Stabilized |
| 3. 저속 푸셔 | Position 모드로 2–3 m/s 직선, 푸셔 보조 | 목표 속도에 닿음, 기울기 10° 미만 | Stabilized, 착륙 |
| 4. Offboard, 멈춰서 돌기 3 m/s | 탑재 노드로 정사각형, 무풍 | 경로 이탈과 tilt가 SITL과 비슷함 | 조종자가 Position, 착륙 |
| 5. 통과 비행 5 m/s | 모서리 roll(SITL 14°), 착지 속력(0.3–0.7 m/s) | SITL과 비슷함 | 같음 |
| 6. 바람을 늘림 | 정풍의 푸셔, 측풍의 yaw | 평균 6 m/s까지 | 같음 |

eCalc의 호버 10.6분은 계산값이므로, 3단계부터 실제 비행 시간을 재서 착륙 여유를 정합니다.

## 3. 그 다음

- **yaw 권한.** 모자라면 모터를 기울여 다는 것(cant)이 첫 후보입니다([교훈 1](06_lessons.md#기체)). 수직 효율과
  축 사이의 결합이 바뀌므로 기하·할당·실측을 함께 바꿔 검증합니다.
- **푸셔 추력선 오프셋.** `CA_ROTOR4_PZ`만 바꾸지 말고 실제 CG, 힘 단위, 축별 정규화, 포화까지 함께 봅니다.
- **저고도 동작.** 푸셔 보조가 5 m 아래에서 꺼질 때 자세 제한이 어떻게 맞물리는지 비행 로그로 확인합니다.
- **SITL에서 아직 못 한 고장.** GPS 상실, 리프트 모터 하나의 고장, 센서 고장, 통신 지연. PX4 v1.17의 Gazebo
  연동으로는 주입할 수 없어 플러그인이나 펌웨어 쪽 작업이 필요합니다.
- **공력 데이터.** 가정한 계수를 측정값으로 바꾸고 로터–날개–평판 간섭을 넣습니다(측정 → 저차 모델 → 필요한
  곳만 CFD).

## 4. custom PX4

조종면 없이 로터 다섯 개로만 나는 quadplane은 실제로 날았습니다. XINCA는 우리와 같은 구성으로 포화를 고려한
할당을 써서 이륙, 전후 이동, 착륙을 했습니다[1]. ArduPilot에도 호버 중 기수를 숙이는 대신 전진 모터를 쓰는
기능이 있습니다([`Q_FWD_THR_USE`](https://ardupilot.org/plane/docs/quadplane-weathervaning.html)). custom PX4를
고민하는 이유는 조종면이 없어서가 아니라, stock의 푸셔 보조가 다음을 하지 않기 때문입니다
([`vtol_type.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/vtol_att_control/vtol_type.cpp),
[`ActuatorEffectivenessStandardVTOL.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessStandardVTOL.cpp)).

- 대기속도나 날개 양력에 따라 힘을 나누지 않습니다.
- 푸셔가 0.9에서 잘린 뒤 모자란 힘을 pitch로 돌리지 않습니다.
- 날개가 무게를 들어 yaw 여유가 줄어도 속도를 제한하지 않습니다.
- 푸셔의 반토크와 오프셋을 할당에 넣지 않습니다.

모터가 계속 돈다고 토크 여유가 있는 것은 아닙니다. 수평 비행에서 `날개 양력 + 리프트 추력 = 무게`인데 날개가
이미 무게만큼 들면, 수직 힘을 깨지 않고 리프트 추력을 더할 수 없습니다. 그래서 첫 목표는 **요구한 힘과 모멘트를
동시에 낼 수 있는 범위를 계산하고, 명령을 그 안으로 제한하는 것**입니다. 같은 모델, 같은 케이스로 stock과
비교하며 한 단계씩 갑니다.

| 단계 | 내용 | 손댈 곳 |
|---|---|---|
| A. 비행 범위 보호 | 대기속도와 날개 힘을 추정하고(유효하지 않으면 낮은 속도로 제한), 푸셔가 모자라면 pitch로 보충하거나 속도를 낮추고, 리프트 여유로 속도와 감속을 제한. 달성하지 못한 힘을 anti-windup에 알리고 ULog에 남김 | `vtol_att_control/vtol_type.cpp`, `standard.cpp`, `mc_pos_control` |
| B. N 단위 할당 | 모터별 실제 추력, 위치, 반토크, 한계, 지연으로 효과 행렬을 만들고 제약 최소제곱으로 할당. 수직 힘과 roll/pitch를 먼저 지킴 | `control_allocator/`, `lib/control_allocation/` |
| C. INDI/XINCA 또는 MPC | 공력·추진 지도와 B의 단위·제약·로그가 갖춰진 뒤 검토 | |

필요한 데이터는 실제 CG와 관성, 리프트·푸셔의 추력·반토크·응답(전압, 대기속도별), 장착 상태의 전후 로터
추력, 날개 CL/CD/Cm, 동체·붐의 6축 하중(받음각·옆미끄럼별), 센서 지연과 잡음입니다.

**CFD는 지금 필요하지 않습니다.** 가장 큰 공백은 CG, 추력, 저 Reynolds 수 날개 계수이고, 이것 없이 CFD를 더하면
잘못된 추진기와 CG를 정밀하게 계산하게 될 뿐입니다. 시위 0.25 m에서 Reynolds 수는 10 m/s에 약 1.7×10⁵,
18 m/s에 약 3.0×10⁵이므로 이 범위의 실측 자료[2]가 먼저입니다. CFD가 필요해지면 기체만 → 로터 disk 포함 →
회전 모델 순으로 가고, 결과는 SITL에 solver를 넣지 말고 계수 지도와 오차 범위로 옮깁니다.

선행 연구:

| 자료 | 관련 있는 부분 | 그대로 쓸 수 없는 이유 |
|---|---|---|
| Karssies & De Wagter, 2022[1] | 조종면을 할당에서 빼고 로터 5개로 비행 | 실내 저속 |
| [Govdeli 외, 2019](https://doi.org/10.1016/j.ast.2019.105421) | 조종면 없는 푸셔 구성을 PID로 제어 | 3자유도, 실비행 없음, 푸셔 역추력 사용 |
| [Lifting-wing quadcopter, 2023](https://arxiv.org/html/2301.00730v1) | 날개 양력과 로터 추력의 통합 제어, 최적화 할당 | 다른 구성, HIL |
| [Mathur & Atkins, 2023](https://arxiv.org/abs/2301.12316) | 유입에 따른 동적 추력, 간섭 항력, 기체 전체 힘 | 다른 기체, 옆미끄럼 없음 |
| [Wechtler 외, 2025](https://research.tudelft.nl/en/publications/implications-of-propeller-wing-interactions-on-the-control-of-aer/) | 날개 간섭으로 앞 로터의 roll 효과가 뒤의 1/2–1/4 | tilting 형상 |

1. H. J. Karssies, C. De Wagter, "Extended Incremental Non-linear Control Allocation (XINCA) for Quadplanes,"
   *International Journal of Micro Air Vehicles*, 2022. [doi:10.1177/17568293211070825](https://doi.org/10.1177/17568293211070825)
2. UIUC, [Low-Speed Airfoil Tests](https://m-selig.ae.illinois.edu/uiuc_lsat.html).

## 5. Vision VTOL로 가는 길

최종 목표는 vision으로 정밀 착륙하는 VTOL입니다. 관련 코드는 [PSI-vtol-team](https://github.com/devkyber/PSI-vtol-team)에
있고, 이 저장소는 그 코드가 아래 자리에 들어오도록 구성했습니다.

| 들어올 것 | PSI-vtol-team의 위치 | 이 저장소에서의 자리 |
|---|---|---|
| 표적 검출, 깊이, 좌표 변환 | `ros2/ws/src/drone_vision/` | 새 패키지 `ros2/ws/src/vtol_vision/`. `/target/ned`를 냄 |
| 정밀 착륙 제어 | `drone_control/vertiport_tracking.py` | `SetpointSource`를 상속한 새 단계. `STEPS`에서 `LANDING` 자리에 넣음 |
| 카메라가 달린 시뮬레이션 기체 | `px4/patches/patch_standard_vtol_camera.py` | `psi_vtol/sim/gz_model.py`에 카메라 센서 추가 |
| Jetson 카메라 구성 | `jetson/`의 RealSense 패키지와 스크립트 | `deploy/jetson/Dockerfile`에 패키지 추가 |
| RL 잔차 제어 | `landing_rl/`, `landing_mujoco/` | 정밀 착륙 단계 안에서 속도 명령에 더하는 보정 |
| 조종면이 있는 다른 기체 | | `vehicle/<이름>.yaml`을 추가하고 `mission.yaml`의 `vehicle`에 지정 |
