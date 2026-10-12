# 5. 해야 할 일과 향후 과제

## 1. 연말 비행까지 해야 할 일

1. **동체팀 수치 확정.** CG 위치, 날개의 전후 위치와 붙임각, 평판 붙임각, 푸셔의 모터·프로펠러와
   위치, 7 kg이 전비중량인지. 확정된 값은 `vehicle/psi_qp.yaml`에 넣고 상태를 GIVEN으로 바꿉니다.
2. **벤치 측정.** 푸셔와 리프트 모터 하나의 추력 곡선. `VT_FWD_THRUST_SC`와 `THR_MDL_FAC`를
   정하고 eCalc 값을 확인합니다. eCalc는 최대 추력에서 모터 온도 92°C를 경고했습니다.
3. **PX4 airframe 설정 파일 작성.** `px4/params.yaml`의 `concept`와, 측정값으로 바꾼 `per_vehicle`,
   모터 위치를 Pixhawk에 올릴 파일로 만듭니다. `sim_only`는 올리지 않습니다.
4. **ROS 2 노드 SITL 시험 확대.** 기준 기체, 무풍에서는 통과했습니다
   ([결과 보고서](../missions/2026_year_end/report.md) 4절). 바람과 불확실한 값에 대한 스윕은 아직
   MAVLink 스크립트로만 했습니다.
5. **실제 비행 경로 정하기.** 지금 프로파일은 이륙할 때의 기수 방향으로 정해진 시간만큼 날아갑니다
   (순항 10 m/s, 15초면 약 250 m). 착륙 지점을 좌표로 주려면 `psi_vtol/profile.py`에 거리 기준
   감속을 넣어야 합니다.
6. **비행 시험.** 구속 호버 → 호버 튜닝(요 게인 포함) → 저속 푸셔 → 속도를 단계적으로 올림 →
   전체 임무. eCalc의 호버 시간 10.6분이 임무 전체의 상한입니다.

끝난 일: 펌웨어 버전 결정(PX4 v1.17.0), 미션 노드 수정(천이 명령과 고정익 경로 제거, 속도 ramp,
vision 없는 착륙).

## 2. 향후 과제

- **대기속도 기준 속도 상한.** 첫 번째 확장입니다. `/fmu/out/airspeed_validated_v1`로 명령 속도를
  제한해 날개 부담을 40% 아래로 유지합니다. 그 전까지는 예상되는 가장 센 정풍에 맞춰 순항
  속도를 낮게 잡습니다.
- **요 권한 증대.** 측풍 비행이 필요하면 리프트 모터를 몇 도 기울이는 것이 일반적인 방법입니다.
- **추력선 오프셋 feedforward** (`CA_ROTOR4_PZ`). PX4의 stock 푸셔 기체들도 쓰지 않고, 넣으면
  푸셔 명령의 배율이 바뀌는 것으로 보이며, 시뮬레이션상 얻을 것이 적습니다.
- **지면 근처에서 푸셔 보조 끄기** (`VT_FWD_THRUST_EN 1`).
- **공력 모델 개선.** 날개 자체의 모멘트와 후류를 넣습니다.
- **실제 고정익 천이.** 조종면이 필요하고, 이 설계의 범위 밖입니다.

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
