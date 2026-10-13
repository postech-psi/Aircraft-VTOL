# 7. 조종면 없는 quadplane 연구와 custom PX4 선택

조사일: 2026-10-13. **다섯 모터만으로 제어하는 개념에는 근거가 있습니다. 우리 기체의
야외 순항 범위는 아직 미확인입니다.** custom PX4를 고려할 이유는 조종면의 부재 자체보다
날개 양력, 푸셔 포화와 리프트 자세 제어 여유를 함께 관리해야 한다는 데 있습니다.

이번 변경은 시뮬레이터·파라미터·미션 감시 개선입니다. PX4 펌웨어 fork나 새 제어기 구현은
포함하지 않았습니다. 아래 내용은 구현 범위와 비교 시험을 정하기 위한 설계안입니다.

## 1. 연구가 실제로 보여 준 것

| 1차 자료 | 관련성 | 근거의 한계 |
|---|---|---|
| [Govdeli 외, 2019](https://doi.org/10.1016/j.ast.2019.105421), Unsteady aerodynamic modeling and control of pusher and tilt-rotor quadplane configurations | 조종면 없는 pusher 구성, 비정상 공력과 PID 제어를 직접 다룸 | 3자유도 수치 연구. roll/yaw·측풍 실비행 증거가 아니며 역천이에서 푸셔 역추력 사용 |
| [Karssies & De Wagter, 2022](https://journals.sagepub.com/doi/10.1177/17568293211070825), XINCA | 마지막 실험은 조종면을 할당에서 제외한 **5개 로터**로 이륙·전후 이동·착륙 | 실내 CyberZoo의 저속 시험. 우리 기체의 outdoor 10 m/s 검증이 아님 |
| [Lifting-wing Quadcopter Modeling and Unified Control, 2023 preprint](https://arxiv.org/html/2301.00730v1) | 양력과 추력 방향을 함께 다루는 통합 제어 접근 | 기체 구성이 다르고 HIL 결과를 우리 기체의 비행시험으로 볼 수 없음 |
| [QuadPlane 풍동 연구, 2023 preprint](https://arxiv.org/html/2301.12316v1) | 유입 속도·방향에 따른 동적 추력 및 완성 기체 힘/모멘트 측정 | 다른 기체. 계수를 그대로 가져오면 안 됨 |
| [Wechtler 외, 2025](https://research.tudelft.nl/en/publications/implications-of-propeller-wing-interactions-on-the-control-of-aer/) | 조종면 없는 tilting quadplane의 propeller-wing 간섭과 제어 효과 변화 | tilting 형상. 보고된 감소 배율을 우리 고정 리프트 기체에 그대로 적용할 수 없음 |

XINCA 원문은 [TU Delft 공개 PDF](https://pure.tudelft.nl/ws/portalfiles/portal/110997639/17568293211070825.pdf)에서도
읽을 수 있습니다. 이 연구는 외부 루프에 푸셔와 자세 증분을 함께 넣고, 내부 루프에서는
제한된 액추에이터를 배분합니다. CPU 부담 때문에 실험 구성을 나누었다는 점도 설계에
참고할 만합니다. 여기서 얻을 교훈은 "같은 하드웨어가 가능하다"와 "공동 배분을 검토할 가치가
있다"이며, 해당 제어기를 그대로 이식하면 우리 기체가 검증된다는 뜻은 아닙니다.

Govdeli 연구의 역추력은 현재 모델에 없습니다. 우리 푸셔는 0 이상의 추력만 내므로 제동은
리프트 추력 방향과 공력 저항으로 해야 합니다. 이 차이 때문에 제동 때 양력 증가와
최소 모터 여유를 별도로 시험합니다.

## 2. stock PX4로 시작할 수 있는 범위

stock v1.17.0에는 MC에서 푸셔를 쓰는 경로가 있습니다. 먼저 이 기준선으로 낮은 날개 부담,
작은 속도 변화, 충분한 푸셔·리프트 여유를 확인하는 것이 합리적입니다. 센서·추력 모델 오차가
큰 상태에서 제어기부터 복잡하게 만들면 문제 원인을 구분하기 어렵습니다.

다만 stock의 pusher assist에는 다음 제한이 있습니다.

- pitch 요구에서 전진 명령을 만드는 보조 경로이며 대기속도·날개 양력 기반 힘 배분이 아닙니다.
- 푸셔 0.9 제한 이후 부족한 힘을 pitch와 공동으로 다시 최적화하지 않습니다.
- 날개 부담이 증가해 yaw 여유가 줄어드는 상황을 운용 envelope로 관리하지 않습니다.
- standard-VTOL 할당은 기하에 기반하지만 수평 로터의 프로펠러 반토크를 비활성화합니다.
  현재 후보 설정은 푸셔 PY/PZ도 0으로 두므로 해당 오프셋 모멘트는 자세 루프의 외란입니다.

근거는 [pusher-assist 소스](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/vtol_att_control/vtol_type.cpp)와
[standard-VTOL effectiveness 소스](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessStandardVTOL.cpp)입니다.
이는 파라미터만으로 통합 제어가 완성되지 않는다는 설명이며, stock 할당기가 전혀 기하
결합을 처리할 수 없다는 뜻은 아닙니다.

## 3. custom PX4를 단계별로 만든다면

### 단계 A: 기존 안정화 루프를 유지한 envelope 보호

처음에는 EKF, MC 각속도/자세 루프와 commander failsafe를 유지합니다. 별도 활성화
파라미터가 있는 pusher-assist 개선부터 비교합니다.

1. 대기속도·자세·전압·모터 출력과 유효성/신선도를 받습니다. 에어스피드나 모델이 유효하지
   않으면 낮은 속도 범위로 제한하고 경고를 남깁니다. 고속에서 갑자기 일반 MC에 넘기는
   fallback도 위험할 수 있으므로 감속·모드 전환 절차를 먼저 정합니다.
2. 날개 힘과 리프트 추력 잔차를 추정합니다. 초기에는 측정한 CL/CD/Cm 지도와 EKF
   속도·풍속을 사용하고 추정 오차 범위를 함께 계산합니다.
3. 푸셔와 pitch에 요구 힘을 배분합니다. 푸셔가 부족하면 허용 받음각·고도·모터 여유 안에서
   pitch 보조를 쓰거나 속도 요구를 낮춥니다. 적분기 anti-windup에 달성하지 못한 힘을 전달합니다.
4. 리프트 최소 여유, yaw 요구, 예상 제동 중 양력까지 보고 속도/감속도를 제한합니다.
   저고도 inhibit에는 hysteresis와 완만한 명령 변화를 넣습니다.
5. 포화·센서 무효·모델 잔차·fallback 이유를 ULog에 기록하고 stock과 같은 case에서 비교합니다.

핵심 구현 후보는 `src/modules/vtol_att_control/vtol_type.cpp`, `standard.cpp`와
`mc_pos_control`의 힘/자세 setpoint 연결입니다. 파일 하나에서 pitch만 덮어쓰는 방식으로
수직 힘과 anti-windup의 일관성이 깨지지 않는지 먼저 확인해야 합니다.

### 단계 B: 공통 단위의 힘·모멘트 할당

효과 행렬은 모터별 실제 추력 T_i의 N 단위로 구성하는 편이 명확합니다.

```text
Fi = axis_i Ti
Mi = (ri − rCG) × Fi + reaction_sign_i kQi axis_i Ti
wrench = Σ(Fi, Mi) + aerodynamic_wrench
```

리프트 네 개와 푸셔의 위치, 방향, 반토크, 추력 한계와 지연을 함께 반영합니다.
현재 PX4의 normalized 축별 배율을 실제 힘 단위와 연결하는 과정도 필요합니다.
`CA_ROTOR4_PZ` 한 개만 바꿔 이 과정이 완료됐다고 간주하지 않습니다.

할당 목표는 각 축의 오차와 명령 변화량을 가중한 constrained least-squares/QP 형태로
시작할 수 있습니다. 양·음 추력 한계, slew, 장착 손실, 전압과 전진 속도에 따른 Tmax를
제약에 넣습니다. 수직 힘과 roll/pitch를 우선하고 yaw·전진 요구를 낮출 수 있는 정책을
명시합니다. 우선순위는 임무에 맞춰 비교 시험으로 정해야 합니다.

관련 위치는 `src/modules/control_allocator/VehicleActuatorEffectiveness/` 및
`src/lib/control_allocation/`입니다. stock standard-VTOL의 FW 상태에서는 리프트 모터를
멈추는 경로가 있으므로, 무조종면 전용 구성을 추가할 때 MC/FW 상태 의미와 motor-stop mask도
함께 설계해야 합니다. 기존 FW 천이를 켜는 것을 지름길로 사용하지 않습니다.

### 단계 C: INDI/INCA/XINCA 또는 MPC

공력/추진 지도와 단계 B의 단위·제약·로그가 준비된 뒤 검토합니다. INDI 계열은 가속도와
각가속도 증분을 쓰므로 센서 필터와 actuator 응답의 시간 정렬이 중요합니다. MPC는
예측 제동/여유 관리에 유용할 수 있지만 모델·solver 시간·fallback 검증이 더 필요합니다.
어떤 제어기를 쓰더라도 한 방향 푸셔와 약한 yaw의 물리적 한계는 남습니다.

## 4. 최소 추력을 남기면 해결되는가

리프트 모터가 계속 돌아간다는 것과 토크 여유가 있다는 것은 다릅니다. 최소 회전수를 남겨도
이미 날개가 무게를 거의 다 들면 수직 힘 평형과 충돌할 수 있습니다. 단순 수평 근사에서
`L + Tlift = W`인데 L이 W 이상이면 양의 Tlift를 강제로 더할 수 없습니다.
받음각·대기속도를 낮추거나 힘 방향/경로를 바꿔야 합니다.

피치업 제동 중에는 더 불리해질 수 있습니다. custom 제어의 첫 성과는 "모터가 계속 돈다"가
아니라 **동시에 요구한 힘과 모멘트를 낼 수 있는 범위를 계산하고 그 안으로 명령을 제한하는 것**이어야 합니다.
수직 꼬리·motor cant 등 기체 변경도 선택지이지만 현재 형상에 없으므로 개선 효과는 별도
모델과 측정으로 확인해야 합니다.

## 5. 구현 전에 필요한 데이터와 비교 시험

| 먼저 얻을 데이터 | 사용할 곳 |
|---|---|
| 전비중량·실제 CG·관성 | 모멘트 기하, 동역학, 제동 예측 |
| 리프트/푸셔 정적 추력·반토크·응답 vs 명령/전압 | 명령 역변환, Tmax, slew, yaw 여유 |
| 동적 추력 vs airspeed/RPM/inflow 방향 | 푸셔 포화 예측, 상승 추력 여유 |
| 설치 상태 전·후 로터 추력, tail trim, wing CL/CD/Cm | 간섭·공력 외란 모델 |
| 전체 동체/boom의 6축 하중 vs α/β/속도/RPM, 압력 중심 | 동체 외란과 pitch/yaw 안정성, 모델의 body drag 근사 교체 |
| 센서 지연·잡음·airspeed 유효 범위 | 추정·INDI 필터·fallback |

비교는 동일 모델·동일 case에서 stock → 단계 A → 단계 B 순으로 합니다. nominal만이 아니라
정풍의 푸셔 포화, 양쪽 측풍 yaw, 높은 양력 제동, 저전압+장착 손실, 대기속도 무효,
노드/통신 상실, CPU 시간 초과를 포함합니다. 신뢰할 수 있는 다음 이정표는 **새 Gazebo
모델의 통합 빌드·nominal SITL·측정 자료 대조**이며 custom PX4 적용 자체가 완료 기준은 아닙니다.
이 중 통합 빌드와 nominal SITL은 2026-10-13 Docker에서 수행했습니다.
[실행 보고서](../missions/2026_year_end/revised_simulation_report.md)의 결과를 출발점으로 삼되
실측 자료 대조는 여전히 남아 있습니다.

## 6. CFD가 지금 필요한가

**첫 제어 가능성 확인에 full CFD는 필수가 아닙니다.** 지금 가장 큰 공백은 실제 CG·관성,
추력/반토크/응답, 저 Reynolds 수에서의 wing CL/CD/Cm입니다. 이 데이터가 없으면
정교한 CFD를 더해도 잘못된 추진기나 CG를 정확히 시뮬레이션하게 될 수 있습니다.
우선 벤치 측정과 기본 공력 지도, PX4 SITL을 연결하는 편이 유용하다는 것이 이번 설계 판단입니다.

시위 0.25 m와 동점성계수 1.5×10⁻⁵ m²/s를 가정하면 10 m/s에서 Re≈1.7×10⁵,
18 m/s에서 Re≈3.0×10⁵입니다. 이는 계산한 운용 조건이며 측정한 대기 상태는 아닙니다.
같은 에어포일 이름의 고 Reynolds 수 데이터를 그대로 쓰기보다 이 범위의 천이·표면 거칠기와
실속 영향을 확인해야 합니다. [UIUC 저속 풍동 자료](https://m-selig.ae.illinois.edu/uiuc_lsat.html)는
이런 데이터의 출처 예시이며 NACA4415의 정확한 설치 상태 계수를 확보했다는 뜻은 아닙니다.

| 알고 싶은 것 | 먼저 할 방법 | CFD의 역할 |
|---|---|---|
| 모터 다섯 개로 force/moment를 만들 수 있는가 | 기하 기반 할당, 추력 벤치, 6DOF SITL | 필수 아님 |
| 기본 순항·제동에서 얼마나 양력이 생기는가 | Reynolds를 맞춘 polar, 유한 날개 보정, trim/민감도 | 측정이 부족한 영역의 보조 예측 |
| 전후 로터가 날개에 얼마나 가려지는가 | 설치 상태 추력 비교·간단한 유동 시험 | actuator-disk 기반 3D 간섭 분석부터 유용 |
| tail이 어느 후류를 받는가, 고받음각에서 어떤 모멘트가 생기는가 | 완성 기체 force/moment 측정 | 3D 정상/비정상 유동과 설계 비교 |
| 큰 동체·boom의 측력/양력과 압력 중심이 어디인가 | 전체 기체 6축 하중과 α/β sweep | 동체 단독→날개 결합→rotor disk의 3D 하중 지도 |
| rotor wake, blade passage, 급하강/동적 실속 | 목표 현상에 맞춘 시험 | 필요 시 회전 블레이드·비정상 CFD, 계산비와 검증 부담 큼 |

CFD를 시작한다면 CAD·CG·프로펠러 직경/피치·RPM·추력·회전 방향·붙임각을 먼저 확정합니다.
기체만 있는 조건 → 로터 disk 포함 → 필요한 경우 비정상 회전 모델 순으로 비교하고,
격자/시간 간격 수렴, 경계조건과 난류/천이 모델 민감도, 측정값과의 차이를 기록합니다.
그 결과는 실시간 SITL에 CFD solver를 넣기보다 CL/CD/Cm 및 장착 추력 보정 지도와
오차 범위로 옮기는 편이 적합합니다.

NASA의 [multirotor CFD 연구](https://nas.nasa.gov/SC20/demos/demo7.html)도 disk와 실제 회전
모델을 구분하고 실험 데이터로 대조합니다. [CFD verification/validation 안내](https://www.grc.nasa.gov/www/wind/valid/tutorial/overview.html)는
수치 해의 검증과 실제 물리와의 일치도 검증을 구분합니다. CFD 결과 하나를 실제 비행 검증으로
대체하지 않는 이유입니다.

동체 외란에 관한 참고로 [QuadPlane 풍동 논문](https://arxiv.org/html/2301.12316v1)은
기체 구조와 전후 로터 유동 간섭이 pitch 모멘트를 바꾸는 것을 측정했습니다.
다만 그 연구도 sideslip 시험은 수행하지 않았으므로 우리 측풍 yaw 계수의 출처로
사용할 수는 없습니다. 현재 모델의 단순 body drag와 nominal CP=CG는 이 공백을 없애지 않습니다.
