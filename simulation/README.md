# 시뮬레이션 Docker 환경

PX4 v1.17.0 SITL, Gazebo Harmonic, Python 의존성을 담은 Linux 이미지와, 그 안에서 명령을 실행하는
wrapper입니다. 호스트에는 Docker만 있으면 되고, macOS(Apple silicon 포함)에서도 headless로 돌아갑니다.
처음 빌드하는 명령은 [README의 빠른 시작](../README.md#빠른-시작)에, 시뮬레이션이 무엇을 계산하고 어떻게
쓰는지는 [4. 시뮬레이션](../docs/04_simulation.md)에 있습니다.

## 이미지

Docker Hub에서 base image를 받지 못하면 공개 mirror를 지정합니다.

```bash
docker build --build-arg BASE_IMAGE=mirror.gcr.io/library/ubuntu:24.04 -f simulation/Dockerfile -t psi-vtol-sitl:v1.17.0-harmonic simulation
```

## 동작 방식

- `docker_run.sh` 뒤에 명령을 붙이면 같은 이미지의 새 컨테이너에서 실행됩니다.
- repo가 컨테이너의 `/workspace`에 마운트되므로 Python 수정은 이미지를 다시 빌드하지 않아도 반영됩니다.
  C++ 플러그인을 고쳤다면 cmake 빌드만 다시 합니다.
- 이미지 안의 PX4는 `/opt/px4`에 있습니다.
- 결과는 호스트의 `out/sitl-runs/`(케이스별 실행 파일)와 `out/<mission>/aeroprop-v2/`(스윕 표)에 남아서
  컨테이너가 끝나도 지워지지 않습니다. `out/`은 git에 올리지 않으므로 보존할 결과는 따로 관리합니다.
- 결과 JSON 안의 `/workspace/...` 경로는 호스트에서는 repo 최상위로 바꿔 읽으면 됩니다.
- Ubuntu, apt, pip 패키지를 버전으로 고정하지 않았습니다. 대신 실행마다 manifest에 이미지 ID, commit,
  패키지 버전, 소스 hash가 남으니 결과와 함께 보존합니다.

## custom PX4 checkout 쓰기

호스트의 PX4 checkout을 절대 경로로 지정하면 컨테이너의 `/opt/px4` 자리에 마운트됩니다. 같은 이미지
안에서 빌드한 뒤 돌립니다.

```bash
export PSI_PX4_CHECKOUT=/absolute/path/PX4-Autopilot-custom
bash simulation/docker_run.sh make -C /opt/px4 px4_sitl_default -j4
bash simulation/docker_run.sh
```

stock으로 돌아가려면 `unset PSI_PX4_CHECKOUT`. runner가 펌웨어를 수정하거나 빌드해 주지는 않습니다.

## 이 이미지로 할 수 없는 것

- **Pixhawk용 펌웨어 빌드.** `--no-nuttx`로 준비한 SITL 개발 환경입니다. `.px4` 파일을 만들려면 대상
  보드의 NuttX toolchain이 따로 필요합니다.
- **ROS 2 노드 시험.** 이미지에 ROS 2가 없고, `docker_run.sh`는 매번 새 컨테이너를 만들어서 DDS 네트워크를
  구성하지 않습니다. MAVLink 시험은 PX4, Gazebo, runner가 한 컨테이너 안에서 통신하므로 문제없습니다.
  ROS 2 시험은 [4. 시뮬레이션 7절](../docs/04_simulation.md#7-ros-2-노드로-비행하기)을 따릅니다.
