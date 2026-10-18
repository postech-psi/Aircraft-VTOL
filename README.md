# Aircraft-VTOL

PSI VTOL 기체의 비행 소프트웨어, PX4 설정, 시뮬레이션을 모아 둔 저장소입니다. 지금은 2026년 연말 비행을
준비하고 있습니다.

기체는 리프트 로터 4개와 푸셔 1개를 단 quadplane이고 조종면이 없습니다. 그래서 고정익으로 천이하지 않고,
이륙부터 착륙까지 멀티콥터 모드로 날면서 앞으로 갈 때 푸셔를 씁니다. 기본 임무는 한 변 50 m 정사각형을
5 m/s로 돌아 이륙 지점에 착륙하는 것입니다. 모서리는 멈추지 않고 반경 15 m 곡선으로 돌고, 마지막 경유점에
정확히 멈춘 뒤 바람을 마주 보고 내려앉습니다.

## 현재 상태

- 비행 제어는 stock PX4 v1.17.0을 쓰고 파라미터만 바꿉니다.
- PX4 SITL + Gazebo에서 기본 임무를 순항 3–7 m/s, 두 경로 방식, 돌풍 섞인 6 m/s 바람으로 비행해 모두
  통과했고, 임무 중단, Jetson 정지, 푸셔 고장, 배터리, 천이 명령에서도 모두 넘어지지 않고 착지했습니다
  ([시뮬레이션 보고서](missions/2026_year_end/report.md)).
- 기체 수치의 상당수가 아직 가정값입니다. 실제 기체를 날리거나 벤치에서 측정한 적은 없습니다.

## 먼저 알아 둘 것

[6. 교훈](docs/06_lessons.md)에서 가장 중요한 다섯 가지입니다.

1. **yaw가 가장 약한 축이고 게인으로는 늘지 않습니다.** 실기체에서 먼저 잽니다.
2. **푸셔는 비행 중 추력으로 정합니다.** 전진 속도에서는 정지 추력의 절반 아래로 떨어질 수 있습니다.
3. **바람을 마주 보고 이륙합니다.** 평균 6 m/s를 넘으면 날지 않습니다.
4. **리프트 모터 하나를 잃으면 자세를 지킬 수 없습니다.** 비행마다 점검하고 사람에게서 멀리 떨어져 납니다.
5. **시뮬레이션은 비교용입니다.** CG, 푸셔 추력, 로터 효율을 잰 뒤 한계값을 다시 정합니다.

## 빠른 시작

Docker만 있으면 됩니다. repo 최상위에서 실행합니다.

```bash
docker build -f simulation/Dockerfile -t psi-vtol-sitl:v1.17.0-harmonic simulation
bash simulation/docker_run.sh cmake -S simulation/gazebo -B simulation/gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
bash simulation/docker_run.sh cmake --build simulation/gazebo/build -j2
bash simulation/docker_run.sh python3 -m pytest -q
bash simulation/docker_run.sh              # 기준 기체로 임무를 한 번 비행
```

첫 이미지 빌드는 PX4를 컴파일하므로 시간이 꽤 걸리고 디스크도 수 GB를 씁니다. 이미지 옵션은
[simulation/README.md](simulation/README.md)에 있습니다.

## 문서

| 문서 | 내용 |
|---|---|
| [0. 전체 구조](docs/00_architecture.md) | 시스템, 코드, 탑재 소프트웨어, 임무 흐름을 그림으로 |
| [1. 기체](docs/01_vehicle.md) | 제원, 무엇으로 무엇을 제어하는가, 수치의 출처 |
| [2. 비행 방법](docs/02_flight.md) | 임무, 경로 계획, PX4에 보내는 것, 착륙, 고장 대응, 푸셔 보조 |
| [3. PX4 설정](docs/03_px4_setup.md) | 버전, 모터 배치, 파라미터, 천이 막기, 실기체에 올리기 |
| [4. 시뮬레이션](docs/04_simulation.md) | 실행 방법, 물리 모델, 합격 기준, 스윕 |
| [5. 탑재 소프트웨어](docs/05_onboard.md) | ROS 2 노드와 토픽, watchdog, Jetson |
| [6. 교훈](docs/06_lessons.md) | 시뮬레이션과 문헌에서 배운 것, 그래서 하는 것 |
| [7. 할 일](docs/07_roadmap.md) | 연말 비행까지, 비행 시험 순서, custom PX4, vision VTOL |
| [시뮬레이션 보고서](missions/2026_year_end/report.md) | 2026-10-18 SITL 결과로 정할 것: 경로 방식, 순항 속도, 바람 한계, 고장 대응 |

## 폴더 구조

| 폴더 | 내용 |
|---|---|
| `vehicle/` | 기체 수치. 기체마다 YAML 하나 |
| `px4/` | PX4 파라미터 |
| `missions/` | 임무마다 폴더 하나: 비행 프로파일, 시뮬레이션 범위, 보고서와 그림 |
| `psi_vtol/` | Python 코드. 비행 프로파일(`profile.py`), 경로 계획(`trajectory.py`), 시뮬레이션(`sim/`), 그림. ROS에 의존하지 않음 |
| `simulation/` | Docker 환경과 Gazebo 공력 플러그인(C++) |
| `ros2/ws/src/vtol_mission/` | Jetson에서 도는 ROS 2 미션 노드 |
| `deploy/` | Jetson용 Docker 구성 |
| `tests/` | PX4 없이 도는 단위 시험 |
| `docs/` | 문서 |

## 수정할 때

같은 종류의 숫자는 한 파일에만 둡니다. 문서의 표는 설명용이고, 코드가 읽는 값은 아래 파일에 있습니다.

| 바꾸려는 것 | 고칠 파일 |
|---|---|
| 기체 치수, 질량, 모터 | `vehicle/psi_qp.yaml` |
| PX4 파라미터 | `px4/params.yaml` |
| 고도, 순항 속도, 가속 한계, 경로(경유점) | `missions/<임무>/mission.yaml` |
| 스윕 범위 | `missions/<임무>/study.yaml` |
| 비행 순서, 고장 판단 | `psi_vtol/profile.py` (시뮬레이션과 ROS 2 노드가 함께 씀) |
| 경로 모양, 속도 계획 | `psi_vtol/trajectory.py` |
| 공력·추진 수식 | `simulation/gazebo/AeroPhysics.hh` |
| PX4 토픽 이름 | `ros2/ws/src/vtol_mission/vtol_mission/px4.py` |
| 임무 단계 | `ros2/ws/src/vtol_mission/launch/mission.launch.py`의 `STEPS` |

- 새 임무는 `missions/<이름>/` 폴더를 추가해서 시작합니다.
- 코드와 주석은 영어, 문서는 한국어로 씁니다.
- 고친 뒤에는 단위 시험을 돌리고, 비행 로직을 건드렸다면 `bash simulation/docker_run.sh`로 한 번 비행시켜
  봅니다. push와 pull request마다 GitHub Actions가 단위 시험과 pyflakes를 Python 3.10, 3.12에서 돌립니다
  ([`tests.yml`](.github/workflows/tests.yml)). PX4 없이 돌기 때문에 `pip install -e ".[dev]"`만 하면 로컬에서도
  `python -m pytest -q`로 같은 시험을 돌릴 수 있습니다.
