# 0. 전체 구조

이 저장소가 어떻게 짜여 있고, 비행할 때 무엇이 무엇에게 명령을 보내는지 그림으로 정리했습니다.
각 부분의 자세한 내용은 번호가 붙은 다른 문서에 있습니다.

## 1. 한눈에 보기

같은 비행 프로파일 코드(`psi_vtol/profile.py`)를 시뮬레이션과 실제 기체가 함께 씁니다. 시뮬레이션은
MAVLink로 PX4 SITL에 명령을 보내고, 실제 기체에서는 Jetson의 ROS 2 노드가 uXRCE-DDS로 Pixhawk의
PX4에 보냅니다. PX4는 어느 쪽이든 수정하지 않은 v1.17.0입니다.

```mermaid
flowchart TB
    subgraph DATA["설정 파일 (숫자는 종류마다 한 곳)"]
        V["vehicle/psi_qp.yaml<br/>기체 치수·질량·모터"]
        M["missions/&lt;임무&gt;/mission.yaml<br/>고도·속도·경로"]
        S["missions/&lt;임무&gt;/study.yaml<br/>바람·스윕 범위"]
        PX["px4/params.yaml<br/>PX4 파라미터"]
    end

    PROF["psi_vtol/profile.py<br/>비행 프로파일: 매 순간의 setpoint"]

    subgraph SIM["시뮬레이션 (Docker, PX4 SITL + Gazebo)"]
        RUN["psi_vtol/sim/run_case.py<br/>MAVLink runner"]
        SITL["PX4 v1.17.0 SITL"]
        GZ["Gazebo Harmonic<br/>+ 공력 플러그인 AeroSystem"]
        MET["psi_vtol/sim/metrics.py<br/>ULog 분석·합격 판정"]
    end

    subgraph REAL["실제 기체"]
        ROS["ROS 2 노드 (Jetson)<br/>ros2/ws/src/vtol_mission"]
        FC["PX4 v1.17.0 (Pixhawk)"]
        MOT["모터 5개<br/>리프트 4 + 푸셔 1"]
    end

    M --> PROF
    V --> RUN
    S --> RUN
    PX --> RUN
    PROF --> RUN
    PROF --> ROS
    RUN -- "MAVLink setpoint" --> SITL
    SITL <-- "모터 명령 / 센서" --> GZ
    SITL -- "ULog" --> MET
    GZ -- "aero.csv" --> MET
    ROS -- "uXRCE-DDS setpoint" --> FC
    FC -- "PWM/DShot" --> MOT
    PX -. "후보 파일 (px4_setup.py), QGC로 적용" .-> FC
```

## 2. 코드 지도

화살표는 "이 모듈이 저 모듈을 쓴다"는 뜻입니다. `psi_vtol`은 ROS에 의존하지 않고, ROS 2 패키지가
`psi_vtol`의 프로파일을 가져다 씁니다.

```mermaid
flowchart LR
    subgraph PKG["psi_vtol (Python, ROS 무관)"]
        CFG["config.py<br/>YAML을 읽어 합침"]
        PROF["profile.py<br/>비행 단계와 setpoint"]
        GZM["sim/gz_model.py<br/>SDF·월드·PX4 기하 생성"]
        RC["sim/run_case.py<br/>케이스 하나 비행"]
        MET["sim/metrics.py<br/>지표·판정"]
        SW["sim/sweep.py<br/>여러 케이스 병렬"]
        ST["sim/start.py<br/>ROS 2 시험용 PX4 기동"]
        PL["plots.py<br/>그림"]
        PS["px4_setup.py<br/>QGC 후보 파일"]
    end

    subgraph CPP["simulation/gazebo (C++)"]
        AP["AeroPhysics.hh<br/>공력·추진 수식"]
        AS["AeroSystem.cc<br/>Gazebo 플러그인"]
    end

    subgraph ROSPKG["vtol_mission (ROS 2)"]
        MM["mission_manager.py"]
        MUX["setpoint_mux.py"]
        SRC["source.py<br/>단계 노드의 공통 부모"]
        PN["path_node.py"]
        LN["landing_node.py"]
        P4["px4.py<br/>토픽 이름·QoS"]
    end

    SW --> RC
    RC --> CFG
    RC --> PROF
    RC --> GZM
    RC --> MET
    ST --> RC
    ST --> GZM
    PL --> MET
    PL --> GZM
    PS --> GZM
    PS --> CFG
    AS --> AP
    GZM -. "플러그인 설정을 SDF로" .-> AS
    PN --> SRC
    LN --> SRC
    PN --> PROF
    LN --> PROF
    PN --> CFG
    LN --> CFG
    SRC --> P4
    MUX --> P4
```

## 3. 탑재 비행 소프트웨어

Companion computer는 "어디로 얼마나 빨리"만 정하고, 자세 제어와 모터 배분은 PX4가 합니다. PX4에
setpoint를 보내는 노드는 `setpoint_mux` 하나뿐입니다.

```mermaid
flowchart LR
    subgraph JETSON["Jetson (ROS 2 Humble)"]
        MM["mission_manager<br/>단계 순서, FAILSAFE"]
        PN["path_node<br/>PATH: 상승·경로"]
        LN["landing_node<br/>LANDING: 하강·착륙"]
        MUX["setpoint_mux<br/>선택된 단계만 통과,<br/>끊기면 HOLD"]
    end

    AG["Micro XRCE-DDS Agent<br/>(시리얼)"]

    subgraph PIXHAWK["Pixhawk (PX4 v1.17.0)"]
        EKF["EKF2<br/>상태 추정"]
        POS["위치·속도 제어<br/>+ 푸셔 보조"]
        ATT["자세·각속도 제어"]
        ALLOC["Control allocation<br/>CA_ROTOR0..4"]
    end

    L["리프트 모터 4개"]
    PU["푸셔"]

    MM -- "/mission/state" --> PN
    MM -- "/mission/state" --> LN
    MM -- "/setpoint_mux/source" --> MUX
    PN -- "/path/trajectory_setpoint" --> MUX
    LN -- "/landing/trajectory_setpoint" --> MUX
    PN -- "/path/status" --> MM
    LN -- "/landing/status" --> MM
    MUX -- "trajectory_setpoint<br/>offboard_control_mode" --> AG
    AG --> POS
    EKF -- "vehicle_local_position<br/>vehicle_status" --> AG
    AG -. "위치·상태" .-> PN
    AG -. "위치·상태" .-> LN
    AG -. "위치" .-> MUX
    EKF --> POS
    POS --> ATT
    ATT --> ALLOC
    ALLOC --> L
    ALLOC --> PU
```

## 4. 임무의 흐름

`mission_manager`는 단계(노드) 단위로, `profile.py`는 그 안의 비행 단계(phase) 단위로 진행합니다. PATH
안에서는 경로를 계획된 기준점을 따라 납니다. 통과 비행(기본)은 가속한 뒤 순항과 모서리(`corner`)를 번갈아
지나 마지막 구간에서 감속하고, 멈춰서 돌기는 경유점마다 회전–가속–순항–감속–호버를 반복합니다. 단계
이름에는 구간 번호가 붙습니다(`accel1`, `corner1`, `hover5`, `turn2`, …). 단계마다 시간 상한이 있고,
넘기거나, 프로파일이 경로를 따라가지 못함을 알아채거나, PX4가 offboard에서 빠지면 `FAILED`를 보고해
`FAILSAFE`로 갑니다. FAILSAFE에서는 mux가 기체를 부드럽게 멈추고, 그 뒤 PX4의 Hold(푸셔 고장이면
Land)에 넘깁니다.

```mermaid
stateDiagram-v2
    direction TB
    [*] --> WAIT_START
    WAIT_START --> PATH: /mission/start
    state PATH {
        direction TB
        climb --> hover1
        hover1 --> accel
        accel --> cruise
        cruise --> corner: 통과 비행
        corner --> cruise
        cruise --> decel: 멈출 경유점 앞
        decel --> hover: 기준점이 경유점에 멈춤
        hover --> turn: 멈춰서 돌기, 다음 경유점이 있음
        turn --> accel: 기수가 다음 구간을 향함
    }
    PATH --> LANDING: 마지막 경유점에서 DONE
    state LANDING {
        direction LR
        align --> descend: 0.3 m 안, 바람을 마주 봄
        descend --> land: 지면 4 m 아래
    }
    LANDING --> MISSION_DONE: disarm
    PATH --> FAILSAFE: FAILED / 응답 없음
    LANDING --> FAILSAFE: FAILED / 응답 없음
    WAIT_START --> FAILSAFE: /mission/abort
```

| 단계 | 하는 일 |
|---|---|
| climb, hover1 | 이륙 지점 위로 목표 고도까지 상승하고 잠시 호버 |
| turn*i* | 멈춰서 돌기: 다음 경유점 쪽으로 기수를 돌림 (호버 상태, 20°/s, 기수가 실제로 돌 때까지) |
| accel*i*, cruise*i*, decel*i* | 구간 *i*를 기준점을 따라 가속–순항–감속. 감속은 경유점에서 정확히 멈추도록 계산됨 |
| corner*i* | 통과 비행: 구간 *i*에서 *i+1*로 반경 15 m 곡선, 옆으로 약 10° 기울여 돎 |
| hover*j* | 멈춘 경유점 위에서 호버 |
| align | 착륙 지점 위에 자리를 잡고 기수를 바람 쪽으로 돌림 |
| descend, land | 착륙 지점(기본은 이륙 지점) 위로 하강, PX4 Land 모드로 착륙 |

## 5. 시뮬레이션 한 번의 흐름

```mermaid
sequenceDiagram
    autonumber
    participant U as 사용자
    participant R as run_case.py
    participant G as gz_model.py
    participant P as PX4 SITL
    participant Z as Gazebo + AeroSystem
    participant M as metrics.py

    U->>R: python -m psi_vtol.sim.run_case --set ...
    R->>G: 케이스 값으로 SDF·월드·환경 생성
    R->>P: PX4 실행 (airframe 4004)
    P->>Z: Gazebo 시작, 기체 스폰
    R->>P: 파라미터 설정 후 응답값 확인
    loop 매 순간 (MAVLink)
        P-->>R: 위치·자세·상태
        R->>P: Profile의 setpoint
        P->>Z: 모터 명령
        Z-->>P: 센서 (공력·추진 손실 포함)
    end
    R->>P: 착륙 후 종료
    R->>M: ULog + aero.csv
    M-->>R: 지표, 합격/실패와 이유
    R-->>U: result.json, manifest.json
```

PX4는 Gazebo의 시계를 따라가지만, Gazebo는 PX4를 기다리지 않습니다. PX4 v1.17의 Gazebo bridge에는
lockstep이 없습니다(`src/modules/simulation/gz_bridge`). 그래서 컴퓨터가 바빠 PX4가 뒤처지면 모터 명령이
시뮬레이션 시간으로 늦게 도착하고, 실제 기체에는 없는 제어 지연이 생깁니다. 목표 배속은 실제의 4배입니다.
센서 잡음과 돌풍의 seed는 고정하지 않아서 같은 케이스도 결과가 조금씩 다릅니다.
