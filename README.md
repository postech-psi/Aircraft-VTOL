# Aircraft-VTOL

PSI VTOL 기체의 제어 설계, 시뮬레이션, 탑재 소프트웨어를 모아 둔 저장소입니다. 지금은 2026년
연말 비행을 준비하고 있습니다.

기체는 리프트 로터 4개와 푸셔 1개를 단 quadplane이고 조종면이 없습니다. 그래서 고정익으로
천이하지 않고, 이륙부터 착륙까지 멀티콥터 모드로 날면서 앞으로 갈 때 푸셔를 씁니다.

## 현재 상태

- 비행 제어는 stock PX4 v1.17.0을 쓰고 파라미터만 바꿉니다.
- PX4 SITL + Gazebo 시뮬레이션에서 기본 임무와 4 m/s 측풍 임무가 통과했습니다. 조건을 일부러
  겹친 stress 시험 7개는 모두 실패했습니다. 자세한 내용은
  [시뮬레이션 보고서](missions/2026_year_end/report.md)에 있습니다.
- 기체 수치의 상당수가 아직 가정값입니다. 실제 기체를 날리거나 벤치에서 측정한 적은 없습니다.

## 빠른 시작

Docker만 있으면 됩니다([환경 안내](simulation/README.md)). repo 최상위에서 실행합니다.

```bash
docker build -f simulation/Dockerfile -t psi-vtol-sitl:v1.17.0-harmonic simulation
bash simulation/docker_run.sh cmake -S simulation/gazebo -B simulation/gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
bash simulation/docker_run.sh cmake --build simulation/gazebo/build -j2
bash simulation/docker_run.sh python3 -m pytest -q
bash simulation/docker_run.sh              # 기준 기체로 임무를 한 번 비행
```

첫 이미지 빌드는 PX4를 컴파일하므로 시간이 꽤 걸리고 디스크도 수 GB를 씁니다.

## 문서

| 문서 | 내용 |
|---|---|
| [1. 기체](docs/01_vehicle.md) | 제원, 무엇으로 무엇을 제어하는가, 수치의 출처 |
| [2. 제어 개념과 PX4 설정](docs/02_control_concept.md) | 비행 순서, 제어 방법, 모터 배치와 파라미터 |
| [3. 시뮬레이션](docs/03_simulation.md) | 실행 방법, 물리 모델, 합격 기준, 스윕 |
| [4. 탑재 소프트웨어](docs/04_onboard.md) | ROS 2 노드, Jetson 구성 |
| [5. 할 일](docs/05_roadmap.md) | 연말 비행까지 남은 일, 이후 과제, vision VTOL 계획 |
| [6. 연구 배경과 custom PX4](docs/06_research_and_custom_px4.md) | 선행 연구, stock PX4의 한계, custom 제어 단계, CFD |
| [시뮬레이션 보고서](missions/2026_year_end/report.md) | 2026-10-13 Docker SITL 실행 결과 |

이전 모델로 낸 결과와 모델을 바꾸기 전에 한 검토는
[`missions/2026_year_end/archive/`](missions/2026_year_end/archive/)에 보관합니다.

## 폴더 구조

| 폴더 | 내용 |
|---|---|
| `vehicle/` | 기체 수치. 기체마다 YAML 하나 |
| `px4/` | PX4 버전과 파라미터 |
| `missions/` | 임무마다 폴더 하나: 비행 프로파일, 시뮬레이션 범위, 보고서와 그림 |
| `psi_vtol/` | Python 코드. 임무 프로파일(`profile.py`)과 시뮬레이션(`sim/`). ROS에 의존하지 않음 |
| `simulation/` | Docker 환경과 Gazebo 공력 플러그인(C++) |
| `ros2/ws/src/vtol_mission/` | Companion computer에서 도는 ROS 2 미션 노드 |
| `deploy/` | Jetson용 Docker 구성 |
| `tests/` | PX4 없이 도는 단위 시험 |
| `docs/` | 문서 |

## 수정할 때

같은 종류의 숫자는 한 파일에만 둡니다. 문서의 표는 설명용이고, 코드가 읽는 값은 아래 파일에
있습니다.

| 바꾸려는 것 | 고칠 파일 |
|---|---|
| 기체 치수, 질량, 모터 | `vehicle/psi_qp.yaml` |
| PX4 파라미터 | `px4/params.yaml` |
| 고도, 순항 속도, 가속 한계 | `missions/<임무>/mission.yaml` |
| 스윕 범위 | `missions/<임무>/study.yaml` |
| 비행 순서, 속도 ramp | `psi_vtol/profile.py` (시뮬레이션과 ROS 2 노드가 함께 씀) |
| 공력·추진 수식 | `simulation/gazebo/AeroPhysics.hh` |
| PX4 토픽 이름 | `ros2/ws/src/vtol_mission/vtol_mission/px4.py` |
| 임무 단계 | `ros2/ws/src/vtol_mission/launch/mission.launch.py`의 `STEPS` |

- 새 임무는 `missions/<이름>/` 폴더를 추가해서 시작합니다.
- 코드와 주석은 영어, 문서는 한국어로 씁니다.
- 고친 뒤에는 `bash simulation/docker_run.sh python3 -m pytest -q`를 돌립니다. 비행 로직을
  건드렸다면 `bash simulation/docker_run.sh`로 한 번 비행시켜 봅니다.
- push와 pull request마다 GitHub Actions가 단위 시험과 pyflakes를 Python 3.10, 3.12에서 돌립니다
  ([`tests.yml`](.github/workflows/tests.yml)). 이 시험들은 PX4 없이 돌기 때문에
  `pip install -e ".[dev]"`만 하면 로컬에서도 같은 명령으로 돌릴 수 있습니다.

## 출처

[PSI-vtol-team](https://github.com/devkyber/PSI-vtol-team)의 `year-end-flight` 브랜치(`c21c169`)에서
옮겨 왔습니다. 시뮬레이션은 `year_end_flight/`에서, `mission_manager`와 `setpoint_mux`는
`ros2/ws/src/drone_control/`의 `supervision.py`와 `setpoint_mux.py`에서 가져왔습니다. Vision, RL,
고정익 경로 코드는 그 저장소에 남아 있습니다.
