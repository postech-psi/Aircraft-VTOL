# 5. 해야 할 일과 향후 과제

## 1. 연말 비행까지 해야 할 일

1. **동체팀 수치 확정.** CG 위치, 날개의 전후 위치와 붙임각, 평판 붙임각, 푸셔의 모터·프로펠러와
   위치, 7 kg이 전비중량인지. 확정된 값은 `vehicle/psi_qp.yaml`에 넣고 상태를 GIVEN으로 바꿉니다.
2. **벤치·추진 측정.** 리프트와 푸셔의 명령–추력–전류 곡선을 전압별로 측정하고 가능하면
   전진 유입에서도 확인합니다. eCalc 값은 출발 가정입니다. 온도·전류·전압과 안전 여유를
   실제로 확인한 뒤 `THR_MDL_FAC`, 추력 한계와 푸셔 배율을 정합니다.
3. **Docker 기준선 시험.** [실행 방법](03_simulation.md)에 따라 `aeroprop-v2` 플러그인을
   빌드하고 nominal → 결합 corner → 바람 방향 → 불확실성 순으로 확인합니다. 이전 모델의
   [보고서](../missions/2026_year_end/report.md) 통과율은 새 모델의 결과가 아닙니다.
4. **Pixhawk 설정 검토.** `python3 -m psi_vtol.px4_setup`으로 후보 파일을 만드는 기능은
   준비됐습니다. 측정값과 모터 순서를 확인하고 QGC에서 출력·센서·failsafe를 설정합니다.
   `sim_only`는 실기체에 적용하지 않습니다. [제어·설정 문서](02_control_concept.md)를 따릅니다.
5. **ROS 2 통합 재시험.** 기본 Docker 이미지는 MAVLink 시험용이며 ROS 2가 없습니다.
   별도 ROS 환경과 DDS 네트워크를 준비한 뒤 상태 지연·Offboard 상실·HOLD·착륙까지 시험합니다.
   이전 ROS 시험 통과 기록이 새 watchdog과 모델의 통합 검증을 대신하지 않습니다.
6. **경로와 비행 envelope 확정.** 현재 임무는 기수 방향의 속도·시간 기준입니다. 감속 후
   정지한 위치에서 착륙하며 지정 vertiport 복귀 기능은 없습니다. 원하는 착륙 좌표가 있으면
   거리 기반 제동과 유효한 위치 추정을 추가합니다. 10 m/s는 검증된 운용 속도가 아닙니다.
7. **비행 시험.** 구속 호버 → 호버 튜닝 → 저속 푸셔 → 속도와 바람 범위를 단계적으로 확대 →
   전체 임무 순서입니다. eCalc의 10.6분 호버 추정은 실제 endurance나 임무 상한을 증명하지
   않습니다. 임무별 전력·온도·배터리 잔량과 착륙 여유를 측정합니다.

준비된 것: PX4 v1.17.0 기준선, 천이 없는 MC+푸셔 임무, 실제 속도 기반 단계 완료,
상태 watchdog, 더 엄격한 ULog 판정, 공력·추진 손실 모델, 후보 파라미터 export.
실기체와의 모델 검증 및 custom 펌웨어 구현은 남아 있습니다.

## 2. 향후 과제

- **custom PX4의 첫 범위.** 대기속도·양력 추정, 최소 리프트 여유, 푸셔 포화, 제동과
  anti-windup을 함께 다루는 단계 A부터 비교합니다. 현재 40% wing-share 판정은 가정한
  검토 기준이며 실제 최소 yaw 여유를 보장하는 상수가 아닙니다.
- **요 권한 측정.** 전진 속도와 리프트 하중별 yaw 모멘트를 확인합니다. 로터 cant는 후보이지만
  수직 효율과 다른 축의 결합을 바꾸므로 기하·할당·실측을 함께 바꿔 검증해야 합니다.
- **추력선 오프셋 배분.** `CA_ROTOR4_PZ`만 바꾸지 말고 실제 CG, 힘 단위, 축별 정규화와
  포화 제약까지 검토합니다. [custom PX4 문서](07_research_and_custom_px4.md)에 범위를 정리했습니다.
- **저고도 동작.** 후보는 `VT_FWD_THRUST_EN=5`와 `MPC_LAND_ALT1`을 사용합니다. 착륙 중
  푸셔 inhibit와 자세 제한의 결합을 SITL 및 비행 로그로 확인합니다.
- **공력 데이터 확보.** 독립 Cm·실속 blending·로터 유입 손실은 모델에 추가했습니다.
  다음은 가정 계수의 실측 교체와 rotor-wing/tail 간섭입니다. CFD의 우선순위는
  [연구 문서](07_research_and_custom_px4.md)의 측정→저차 모델→필요한 CFD 순서를 따릅니다.
- **날개 중심의 고속 비행.** 조종면 없이도 연구 선례는 있지만, 리프트를 멈추는 stock FW
  천이를 켜면 자세 제어 수단을 잃습니다. 별도 통합 제어와 충분한 actuator 여유가 필요한
  다른 검증 단계입니다.

## 3. Vision VTOL로 가는 길

최종 목표는 vision으로 정밀 착륙하는 VTOL입니다. 이번 비행에서는 뺐고, 관련 코드는
[PSI-vtol-team](https://github.com/devkyber/PSI-vtol-team)에 남아 있습니다. 이 repo의 구조는 그 코드가
아래 자리에 들어오도록 잡았습니다.

| 들어올 것 | PSI-vtol-team의 위치 | 이 repo에서의 자리 |
|---|---|---|
| 표적 검출, 깊이, 좌표 변환 | `ros2/ws/src/drone_vision/` | 새 패키지 `ros2/ws/src/vtol_vision/`. 지금처럼 `/target/ned`를 냅니다. |
| 정밀 착륙 제어 | `drone_control/vertiport_tracking.py` | `SetpointSource`를 상속한 새 단계(예: `PRECISION_LANDING`). `STEPS`에서 `LANDING` 자리에 넣습니다. |
| 카메라가 달린 시뮬레이션 기체 | `px4/patches/patch_standard_vtol_camera.py` | `psi_vtol/sim/gz_model.py`에 카메라 센서 추가. 표적이 있는 월드는 `px4_assets/worlds/psi_vtol_world.sdf`. |
| Jetson의 카메라 구성 | `jetson/`의 RealSense 패키지와 스크립트 | `deploy/jetson/Dockerfile`에 패키지 추가. |
| RL 잔차 제어 | `landing_rl/`, `landing_mujoco/` | 정밀 착륙 단계 안에서 속도 명령에 더하는 보정. |
| 조종면이 있는 다른 기체 | | `vehicle/<이름>.yaml`을 추가하고 `mission.yaml`의 `vehicle`에 적습니다. |

새 임무는 `missions/<이름>/` 폴더를 하나 더 만드는 것으로 시작합니다.
