# 5. 할 일

## 1. 연말 비행까지

1. **동체팀 수치 확정.** CG 위치, 날개의 전후 위치와 붙임각, 평판 붙임각, 푸셔의 모터·프로펠러와 위치,
   7 kg이 전비중량인지. 확정된 값은 `vehicle/psi_qp.yaml`에 넣고 표시를 GIVEN으로 바꿉니다.
2. **추진 벤치 측정.** 리프트 모터와 푸셔의 명령–추력–전류 곡선을 전압별로 잽니다. 가능하면 전진
   유입이 있는 상태에서도 잽니다. 온도·전류·전압 여유를 확인한 뒤 `THR_MDL_FAC`, 추력 한계,
   `VT_FWD_THRUST_SC`를 정합니다.
3. **시뮬레이션 스윕.** 지금까지는 nominal, 측풍, corner 10건만 돌렸습니다
   ([보고서](../missions/2026_year_end/report.md)). OAT, wind, random 스윕을 `aeroprop-v2`로 돌리고,
   측정값이 들어올 때마다 다시 돌립니다.
4. **Pixhawk 설정.** `python3 -m psi_vtol.px4_setup`으로 후보 파일을 만들고, 측정값과 모터 순서를
   확인한 뒤 QGC에서 출력·센서·failsafe를 설정합니다. `sim_only`는 올리지 않습니다
   ([설정 순서](02_control_concept.md#3-px4-설정)).
5. **ROS 2 통합 시험.** 기본 Docker 이미지에는 ROS 2가 없으므로 ROS 환경과 DDS 네트워크를 따로
   준비합니다. 상태 지연, offboard 상실, HOLD, 착륙까지 시험합니다. 이전 ROS 시험(2026-10-11)은
   지금의 watchdog과 모델로 한 것이 아니므로 다시 해야 합니다.
6. **경로와 비행 범위 확정.** 지금 임무는 기수 방향으로 정해진 속도와 시간만큼 가서, 멈춘 곳에
   착륙합니다. 정해진 지점에 착륙해야 한다면 거리 기준 제동과 위치 유도를 추가합니다. 10 m/s는 아직
   검증된 운용 속도가 아닙니다.
7. **비행 시험.** 구속 호버 → 호버 튜닝 → 저속 푸셔 → 속도와 바람 범위를 단계적으로 확대 → 전체 임무
   순서로 진행합니다. eCalc의 호버 10.6분은 계산값이므로, 비행마다 전력·온도·배터리 잔량과 착륙 여유를
   기록합니다.

이미 된 것: PX4 v1.17.0 기준선, 천이 없는 MC + 푸셔 임무, 실제 속도 도달을 확인하는 단계 전환, 상태
watchdog, ULog 기반 합격 판정, 공력·추진 손실 모델, 후보 파라미터 export.

## 2. 이후 과제

- **custom PX4.** 대기속도·날개 양력 추정, 최소 리프트 여유, 푸셔 포화, 제동, anti-windup을 함께 다루는
  단계 A부터 stock과 비교합니다([6. 연구 배경과 custom PX4](06_research_and_custom_px4.md)). 지금의
  날개 부담 40% 기준은 이 작업에서 실제 yaw 여유에 근거한 제한으로 바꿔야 합니다.
- **yaw 권한 측정.** 전진 속도와 리프트 하중별 yaw 모멘트를 잽니다. 로터를 기울여 다는 것(cant)도
  후보지만 수직 효율과 다른 축의 결합이 바뀌므로 기하·할당·실측을 함께 바꿔서 검증합니다.
- **푸셔 추력선 오프셋.** `CA_ROTOR4_PZ`만 바꾸지 말고 실제 CG, 힘 단위, 축별 정규화, 포화 제약까지
  함께 봅니다.
- **저고도 동작.** `VT_FWD_THRUST_EN=5`와 `MPC_LAND_ALT1` 경계에서 푸셔가 꺼지는 것과 자세 제한이 어떻게
  맞물리는지 SITL과 비행 로그로 확인합니다.
- **공력 데이터.** 가정한 계수를 측정값으로 바꾸고, 로터–날개–평판 간섭을 모델에 넣습니다. 측정 →
  저차 모델 → 필요한 곳만 CFD 순서로 갑니다.
- **날개로 무게를 대부분 드는 고속 비행.** 조종면 없이도 연구 사례는 있지만, stock 고정익 천이는 리프트
  모터를 멈추므로 자세를 잡을 수단이 없어집니다. 별도 통합 제어와 충분한 모터 여유가 필요한 다른
  단계의 일입니다.

## 3. Vision VTOL로 가는 길

최종 목표는 vision으로 정밀 착륙하는 VTOL입니다. 이번 비행에서는 뺐고, 관련 코드는
[PSI-vtol-team](https://github.com/devkyber/PSI-vtol-team)에 남아 있습니다. 이 저장소는 그 코드가 아래
자리에 들어오도록 구성했습니다.

| 들어올 것 | PSI-vtol-team의 위치 | 이 저장소에서의 자리 |
|---|---|---|
| 표적 검출, 깊이, 좌표 변환 | `ros2/ws/src/drone_vision/` | 새 패키지 `ros2/ws/src/vtol_vision/`. 지금처럼 `/target/ned`를 냄 |
| 정밀 착륙 제어 | `drone_control/vertiport_tracking.py` | `SetpointSource`를 상속한 새 단계(예: `PRECISION_LANDING`). `STEPS`에서 `LANDING` 자리에 넣음 |
| 카메라가 달린 시뮬레이션 기체 | `px4/patches/patch_standard_vtol_camera.py` | `psi_vtol/sim/gz_model.py`에 카메라 센서 추가. 표적이 있는 월드는 `px4_assets/worlds/psi_vtol_world.sdf` |
| Jetson 카메라 구성 | `jetson/`의 RealSense 패키지와 스크립트 | `deploy/jetson/Dockerfile`에 패키지 추가 |
| RL 잔차 제어 | `landing_rl/`, `landing_mujoco/` | 정밀 착륙 단계 안에서 속도 명령에 더하는 보정 |
| 조종면이 있는 다른 기체 | | `vehicle/<이름>.yaml`을 추가하고 `mission.yaml`의 `vehicle`에 지정 |

새 임무는 `missions/<이름>/` 폴더를 하나 더 만드는 것으로 시작합니다.
