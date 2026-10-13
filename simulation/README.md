# Docker 시뮬레이션 환경

모든 시뮬레이션 명령은 repo 최상위에서 실행합니다. 호스트에는 Docker만 설치하면 되고
PX4/Gazebo/Python 의존성은 이미지 안에 둡니다. macOS는 Linux VM을 쓰며 headless로 실행합니다.
이 환경의 실제 검증 상태와 물리 모델 범위는 [시뮬레이션 문서](../docs/03_simulation.md)에 있습니다.
2026-10-13 Linux ARM64에서 이미지·플러그인 빌드, 시험 53개, 실제 PX4/Gazebo 비행을 실행했습니다.
통과 조건과 실패 case는 [실행 보고서](../missions/2026_year_end/revised_simulation_report.md)에 보존했습니다.

```bash
docker build -f simulation/Dockerfile -t psi-vtol-sitl:v1.17.0-harmonic simulation
bash simulation/docker_run.sh cmake -S simulation/gazebo -B simulation/gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
bash simulation/docker_run.sh cmake --build simulation/gazebo/build -j2
bash simulation/docker_run.sh python3 -m pytest -q
bash simulation/docker_run.sh
```

한 비행의 그림을 바로 만들려면 결과 JSON 경로를 지정합니다.

```bash
bash simulation/docker_run.sh python3 -m psi_vtol.sim.run_case --case nominal --json /workspace/out/nominal.json
bash simulation/docker_run.sh python3 -m psi_vtol.plots --record out/nominal.json
```

이미지에 PX4 v1.17.0 소스·SITL 빌드와 Gazebo Harmonic을 넣습니다. repo는 `/workspace`로
마운트되므로 소스 변경은 이미지 rebuild 없이 반영됩니다. C++ 수정 후에는 플러그인을
다시 빌드합니다. 최초 이미지 빌드는 다운로드·컴파일 시간과 수 GB의 저장 공간을 사용합니다.
가져온 Ubuntu·apt/Python 패키지는 완전히 lock한 snapshot이 아니므로 실행 manifest의
버전/commit/hash를 함께 보존합니다.

Docker Hub 연결이 어려우면 공개 Ubuntu mirror를 지정할 수 있습니다.

```bash
docker build --build-arg BASE_IMAGE=mirror.gcr.io/library/ubuntu:24.04 -f simulation/Dockerfile -t psi-vtol-sitl:v1.17.0-harmonic simulation
```

추가 명령은 같은 wrapper 뒤에 붙입니다.

```bash
bash simulation/docker_run.sh python3 -m psi_vtol.sim.run_case --set wind_speed_mps=4 --set wind_from_deg=90
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep corners --jobs 2
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep oat --jobs 2
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep wind --jobs 2
bash simulation/docker_run.sh python3 -m psi_vtol.sim.sweep random --n 100 --seed 1 --jobs 2
bash simulation/docker_run.sh python3 -m psi_vtol.plots
bash simulation/docker_run.sh python3 -m psi_vtol.px4_setup --output /workspace/out/psi_candidate.params
```

첫 nominal 실행 후 `out/`이 생깁니다. 결과는 호스트의 `out/sitl-runs/`와
`out/<mission>/aeroprop-v2/`에 남습니다. 컨테이너가 종료돼도 삭제되지 않습니다.
JSON 내부 `/workspace/...` 경로는 같은 이미지에서 다시 열 수 있고 호스트에서는 repo 경로로
바꿔 읽습니다. `out/`은 git ignore 대상입니다. 보존할 결과는 검증 보고서와 함께 따로 관리합니다.

`docker_run.sh`는 매번 새 컨테이너를 만들므로 외부 ROS 2 연결에 필요한 DDS 네트워크를
자동 구성하지 않습니다. 기본 MAVLink 시험은 PX4/Gazebo/runner가 같은 컨테이너 안에서 통신합니다.
ROS 2 시험은 [시뮬레이션 문서](../docs/03_simulation.md)의 별도 통합 경로를 따릅니다.

custom PX4 checkout을 비교하려면 절대 경로로 마운트하고 같은 컨테이너 환경에서 빌드합니다.

```bash
export PSI_PX4_CHECKOUT=/absolute/path/PX4-Autopilot-custom
bash simulation/docker_run.sh make -C /opt/px4 px4_sitl_default -j4
bash simulation/docker_run.sh
```

컨테이너의 `/opt/px4`가 이 checkout으로 바뀝니다. stock으로 돌아갈 때는
`unset PSI_PX4_CHECKOUT`을 실행합니다. 실제 펌웨어 수정은 자동으로 이루어지지 않습니다.
현재 이미지는 `--no-nuttx`로 준비한 **SITL 개발 환경**입니다. Pixhawk에 올릴 `.px4` 파일을
빌드하려면 대상 보드에 맞는 NuttX toolchain과 별도 빌드·벤치 검증이 필요합니다.
