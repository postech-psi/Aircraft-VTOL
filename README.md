# Aircraft-VTOL

PSI VTOL 기체의 제어 설계, 시뮬레이션, 탑재 소프트웨어입니다. 지금 담긴 것은 2026 연말 비행입니다.
리프트 로터 4개와 푸셔 1개로, 이륙부터 착륙까지 멀티콥터로 비행합니다. 조종면이 없으므로 고정익
천이는 하지 않습니다.

시뮬레이션으로만 확인했고 실제 비행은 하지 않았습니다.

## 읽는 순서

1. [기체](docs/01_vehicle.md): 무엇을 제어할 수 있는가
2. [제어 설계](docs/02_control_concept.md): 임무, 제어 방법, PX4 설정
3. [시뮬레이션](docs/03_simulation.md): 준비와 실행
4. [탑재 소프트웨어](docs/04_onboard.md): ROS 2 노드, Jetson
5. [해야 할 일과 향후 과제](docs/05_roadmap.md): vision VTOL로 가는 길 포함
6. [연말 비행 시뮬레이션 결과](missions/2026_year_end/report.md)

## 폴더

| 폴더 | 내용 |
|---|---|
| `vehicle/` | 기체 수치. 기체마다 파일 하나 |
| `px4/` | PX4 버전(v1.17.0)과 파라미터 |
| `missions/` | 임무마다 폴더 하나: 비행 프로파일, 시뮬레이션 범위, 결과 보고서와 그림 |
| `psi_vtol/` | Python 코드. 임무 프로파일(`profile.py`)과 시뮬레이션(`sim/`). ROS에 의존하지 않음 |
| `ros2/ws/src/vtol_mission/` | Companion computer에서 도는 ROS 2 미션 노드 |
| `deploy/` | Jetson용 Docker 구성 |
| `tests/` | PX4 없이 도는 단위 시험 |
| `docs/` | 문서 |

## 빨리 돌려 보기

PX4 v1.17.0과 Gazebo가 준비되어 있어야 합니다([준비](docs/03_simulation.md)). repo 최상위에서
실행합니다.

```bash
python3 -m pytest                          # 단위 시험, PX4 없이
python3 -m psi_vtol.sim.run_case           # 기준 기체로 임무 한 번 비행
```

## 고칠 때

숫자는 종류마다 한 곳에만 있습니다. 문서의 표는 설명이고, 코드가 읽는 값은 아래 파일입니다.

| 바꾸려는 것 | 고칠 파일 |
|---|---|
| 기체 치수, 질량, 모터 | `vehicle/psi_qp.yaml` |
| PX4 파라미터 | `px4/params.yaml` |
| 고도, 순항 속도, 가속 한계 | `missions/<임무>/mission.yaml` |
| 스윕 범위 | `missions/<임무>/study.yaml` |
| 비행 순서, 속도 ramp | `psi_vtol/profile.py` (시뮬레이션과 ROS 2 노드가 함께 씀) |
| PX4 토픽 이름 | `ros2/ws/src/vtol_mission/vtol_mission/px4.py` |
| 임무 단계 | `ros2/ws/src/vtol_mission/launch/mission.launch.py`의 `STEPS` |

- 새 임무는 `missions/<이름>/` 폴더를 추가합니다.
- 코드와 주석은 영어로, 문서는 한국어로 씁니다.
- 고친 뒤에는 `python3 -m pytest`를 돌리고, 비행 로직을 건드렸으면
  `python3 -m psi_vtol.sim.run_case`로 한 번 비행시켜 봅니다.

## 출처

[PSI-vtol-team](https://github.com/devkyber/PSI-vtol-team)의 `year-end-flight` 브랜치(`c21c169`)에서
옮겼습니다. 시뮬레이션은 `year_end_flight/`에서, `mission_manager`와 `setpoint_mux`는
`ros2/ws/src/drone_control/`의 `supervision.py`와 `setpoint_mux.py`에서 왔습니다. Vision, RL,
고정익 경로 코드는 그 repo에 남아 있습니다.
