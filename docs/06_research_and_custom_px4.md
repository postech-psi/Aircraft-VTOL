# 6. 연구 배경과 custom PX4

조사일: 2026-10-13.

모터 다섯 개만으로 제어하는 quadplane에는 연구 근거가 있습니다. custom PX4를 고민하는 이유는 조종면이
없어서라기보다, 날개 양력과 푸셔 포화와 리프트 모터의 자세 제어 여유를 한꺼번에 관리해야 하기
때문입니다. 지금은 펌웨어를 수정하지 않았고, 이 문서는 어떤 순서로 만들고 비교할지에 대한 설계안입니다.

## 1. 선행 연구

| 자료 | 관련 있는 부분 | 우리에게 그대로 적용할 수 없는 이유 |
|---|---|---|
| [Govdeli 외, 2019](https://doi.org/10.1016/j.ast.2019.105421) — pusher·tilt-rotor quadplane의 비정상 공력 모델링과 제어 | 조종면 없는 푸셔 구성을 PID로 제어 | 3자유도 수치 연구. roll/yaw, 측풍, 실비행이 없고 역천이에 푸셔 역추력을 씀 |
| [Karssies & De Wagter, 2022](https://journals.sagepub.com/doi/10.1177/17568293211070825) — XINCA | 조종면을 할당에서 빼고 로터 5개로 이륙·전후 이동·착륙 | 실내 저속 시험 |
| [Lifting-wing quadcopter, 2023](https://arxiv.org/html/2301.00730v1) | 날개 양력과 로터 추력을 함께 다루는 통합 제어와 최적화 기반 할당 | 기체 구성이 다르고 HIL 결과 |
| [QuadPlane 풍동 연구, 2023](https://arxiv.org/html/2301.12316v1) | 유입 속도·방향에 따른 동적 추력, 간섭에 따른 항력 증가와 추력 감소, 기체 전체 힘/모멘트 | 다른 기체. sideslip 시험이 없음 |
| [Wechtler 외, 2025](https://research.tudelft.nl/en/publications/implications-of-propeller-wing-interactions-on-the-control-of-aer/) | 프로펠러–날개 간섭으로 전방 로터의 roll 효과가 후방의 1/2–1/4로 줄어듦. 할당 보정 후 비행 성능 개선 | tilting 형상이라 배율을 그대로 쓸 수 없음 |

여기서 얻을 것:

- **같은 하드웨어로 날 수 있다.** XINCA는 우리와 같은 로터 5개 구성으로 실제로 날았습니다. 외부 루프에서
  푸셔와 자세 증분을 함께 정하고, 내부 루프에서 제한된 모터 출력을 배분하는 구조는 참고할 만합니다
  ([공개 PDF](https://pure.tudelft.nl/ws/portalfiles/portal/110997639/17568293211070825.pdf)).
- **정지 추력만으로는 부족하다.** 풍동 시험에서 전진 중 추력이 줄고 간섭 항력이 늘었습니다. 순항 추력은
  동적 추력 데이터로 정해야 합니다.
- **모터별 효율 차이가 클 수 있다.** 간섭으로 앞뒤 로터의 효과가 몇 배 차이 날 수 있으므로, 모터 간
  불일치 ±8%만 시험하는 것으로는 부족합니다. 그래서 스윕에 전후 로터 장착 효율(0.65–1.0)을 넣었습니다.
- **우리 푸셔는 역추력이 없다.** Govdeli 연구와 달리 제동은 리프트 추력 방향과 공기 저항으로만 합니다.
  그래서 제동 중 양력 증가와 리프트 모터의 최소 여유를 따로 시험합니다.

## 2. stock PX4로 할 수 있는 것

stock v1.17.0에는 MC 상태에서 푸셔를 쓰는 경로가 있습니다(`Standard::update_mc_state()`,
`VtolType::pusher_assist()`). 날개 부담이 낮고 속도 변화가 작으며 푸셔·리프트 여유가 충분한 범위는
이것으로 시작하는 것이 맞습니다. 센서나 추력 모델 오차가 큰 상태에서 제어기부터 복잡하게 만들면
문제가 어디서 왔는지 구분하기 어렵습니다.

다만 stock에는 다음 한계가 있습니다.

- pusher assist는 pitch 요구를 전진 명령으로 바꾸는 보조 경로이고, 대기속도나 날개 양력에 따라 힘을
  배분하지 않습니다.
- 푸셔 명령이 0.9에서 잘린 뒤 모자란 힘을 pitch와 함께 다시 배분하지 않습니다.
- 날개 부담이 커져 yaw 여유가 줄어드는 상황을 막는 속도 제한이 없습니다.
- standard-VTOL 할당기는 푸셔의 반토크를 쓰지 않고, 지금 설정은 푸셔 오프셋(PY/PZ)도 0입니다. 그
  모멘트는 자세 루프가 외란으로 받습니다.

근거: [`vtol_type.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/vtol_att_control/vtol_type.cpp),
[`standard.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/vtol_att_control/standard.cpp),
[`ActuatorEffectivenessStandardVTOL.cpp`](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessStandardVTOL.cpp).

조종면이 없다고 고정익 비행이 원리적으로 불가능한 것은 아닙니다. 로터를 계속 쓰는 설계도 가능하고 PX4에도
고정익 differential thrust 옵션(`VT_FW_DIFTHR_EN`)이 있습니다. 하지만 stock 고정익 천이는 리프트 모터를
멈추므로 이 기체에서는 막아 두는 것이 맞습니다.

## 3. custom PX4를 만든다면

### 단계 A: 기존 안정화 루프 위에 비행 범위 보호

EKF, MC 각속도·자세 루프, commander failsafe는 그대로 두고, 별도 파라미터로 켜는 pusher assist 개선부터
stock과 비교합니다.

1. 대기속도, 자세, 전압, 모터 출력과 그 유효성·신선도를 받습니다. 대기속도나 모델이 유효하지 않으면
   낮은 속도로 제한하고 경고를 남깁니다. 고속에서 갑자기 일반 MC로 넘기는 것도 위험할 수 있으므로
   감속과 모드 전환 절차를 먼저 정합니다.
2. 날개 힘과 리프트 추력의 잔차를 추정합니다. 처음에는 측정한 CL/CD/Cm 지도와 EKF 속도·풍속을 쓰고,
   추정 오차 범위도 함께 계산합니다.
3. 필요한 전진력을 푸셔와 pitch에 배분합니다. 푸셔가 모자라면 허용 받음각·고도·모터 여유 안에서 pitch로
   보충하거나 속도 요구를 낮춥니다. 달성하지 못한 힘은 적분기 anti-windup에 알려 줍니다.
4. 리프트 모터의 최소 여유, yaw 요구, 제동 중 예상 양력을 보고 속도와 감속도를 제한합니다. 저고도에서
   푸셔를 끌 때는 hysteresis를 두고 명령을 완만하게 바꿉니다.
5. 포화, 센서 무효, 모델 잔차, fallback 이유를 ULog에 남기고 stock과 같은 케이스로 비교합니다.

손댈 곳은 `src/modules/vtol_att_control/vtol_type.cpp`, `standard.cpp`, 그리고 `mc_pos_control`의
힘·자세 setpoint 연결입니다. pitch만 덮어쓰다가 수직 힘과 anti-windup의 일관성이 깨지지 않는지 먼저
확인해야 합니다.

### 단계 B: N 단위 힘·모멘트 할당

효과 행렬을 모터별 실제 추력(N)으로 구성합니다.

```text
Fi = axis_i × Ti
Mi = (ri − rCG) × Fi + reaction_sign_i × kQi × axis_i × Ti
wrench = Σ(Fi, Mi) + aerodynamic_wrench
```

리프트 네 개와 푸셔의 위치, 방향, 반토크, 추력 한계, 지연을 모두 넣고, 지금 PX4의 정규화된 축별 배율과
실제 힘 단위를 연결합니다. `CA_ROTOR4_PZ` 하나만 바꾸는 것으로는 이 작업을 대신할 수 없습니다.

할당은 축별 오차와 명령 변화량에 가중치를 둔 제약 최소제곱(QP)으로 시작할 수 있습니다. 추력 상하한,
slew, 장착 손실, 전압과 전진 속도에 따른 Tmax를 제약으로 넣고, 수직 힘과 roll/pitch를 먼저 지키면서
yaw와 전진 요구를 줄이는 우선순위를 명시합니다. 구체적인 우선순위는 임무에 맞춰 비교 시험으로 정합니다.

관련 코드는 `src/modules/control_allocator/VehicleActuatorEffectiveness/`와
`src/lib/control_allocation/`에 있습니다. stock standard-VTOL은 FW 상태에서 리프트 모터를 멈추므로,
조종면 없는 기체용 구성을 추가할 때는 MC/FW 상태의 의미와 motor-stop mask도 함께 설계해야 합니다.

### 단계 C: INDI/INCA/XINCA 또는 MPC

공력·추진 지도와 단계 B의 단위·제약·로그가 갖춰진 뒤에 검토합니다. INDI 계열은 가속도와 각가속도
증분을 쓰므로 센서 필터와 모터 응답의 시간 정렬이 중요합니다. MPC는 제동과 여유 관리를 미리 계산할 수
있지만 모델, solver 시간, fallback 검증이 더 많이 필요합니다. 어느 쪽이든 한 방향뿐인 푸셔와 약한 yaw라는
물리적 한계는 그대로입니다.

### 리프트 모터에 최소 추력을 남기면 되는가

모터가 계속 돈다고 토크 여유가 있는 것은 아닙니다. 수평 비행에서 `L + Tlift = W`인데 날개 양력 L이 이미
무게 W만큼이면, 수직 힘의 균형을 깨지 않고 리프트 추력을 더할 수 없습니다. 이때는 받음각이나 대기속도를
낮추거나 힘의 방향, 경로를 바꿔야 합니다. 기수를 드는 제동에서는 더 불리합니다.

그래서 custom 제어의 첫 목표는 "모터가 계속 돈다"가 아니라 **요구한 힘과 모멘트를 동시에 낼 수 있는
범위를 계산하고, 명령을 그 안으로 제한하는 것**입니다. 수직 꼬리나 motor cant 같은 기체 변경도 방법이지만
지금 형상에는 없으므로 별도 모델과 측정이 필요합니다.

## 4. 구현 전에 필요한 데이터

| 데이터 | 쓰는 곳 |
|---|---|
| 전비중량, 실제 CG, 관성 | 모멘트 기하, 동역학, 제동 예측 |
| 리프트·푸셔의 정적 추력, 반토크, 응답 vs 명령·전압 | 명령 역변환, Tmax, slew, yaw 여유 |
| 대기속도·회전수·유입 방향별 동적 추력 | 푸셔 포화 예측, 상승 추력 여유 |
| 장착 상태의 전후 로터 추력, 평판 trim, 날개 CL/CD/Cm | 간섭과 공력 외란 모델 |
| 동체·붐 전체의 6축 하중 vs 받음각·옆미끄럼·속도·RPM, 압력 중심 | 동체 외란, pitch/yaw 안정성. 시뮬레이션의 축별 항력 근사를 대체 |
| 센서 지연·잡음, 대기속도 센서 유효 범위 | 추정, INDI 필터, fallback |

비교는 같은 모델·같은 케이스로 stock → 단계 A → 단계 B 순서로 합니다. nominal만이 아니라 정풍에서의
푸셔 포화, 양쪽 측풍의 yaw, 높은 양력에서의 제동, 저전압 + 장착 손실, 대기속도 무효, 노드·통신 상실,
CPU 시간 초과를 포함합니다.

## 5. CFD가 지금 필요한가

처음 제어 가능성을 확인하는 데 full CFD는 필요하지 않습니다. 지금 가장 큰 공백은 실제 CG와 관성,
추력·반토크·응답, 낮은 Reynolds 수에서의 날개 CL/CD/Cm입니다. 이 값들이 없는 상태에서 CFD를 더하면
잘못된 추진기와 CG를 정밀하게 시뮬레이션하게 될 뿐입니다. 벤치 측정과 기본 공력 지도를 SITL에 먼저
연결하는 편이 낫습니다.

시위 0.25 m, 동점성계수 1.5×10⁻⁵ m²/s로 계산하면 Reynolds 수는 10 m/s에서 약 1.7×10⁵, 18 m/s에서 약
3.0×10⁵입니다. 높은 Reynolds 수에서 얻은 NACA 4415 데이터를 그대로 쓰기보다 이 범위에서의 천이,
표면 거칠기, 실속을 확인해야 합니다([UIUC 저속 풍동 자료](https://m-selig.ae.illinois.edu/uiuc_lsat.html)).

| 알고 싶은 것 | 먼저 할 일 | CFD의 역할 |
|---|---|---|
| 모터 다섯 개로 필요한 힘과 모멘트를 낼 수 있나 | 기하 기반 할당, 추력 벤치, 6자유도 SITL | 필요 없음 |
| 순항과 제동에서 양력이 얼마나 생기나 | Reynolds 수를 맞춘 polar, 유한 날개 보정, trim 계산 | 측정이 부족한 영역의 보조 예측 |
| 전후 로터가 날개에 얼마나 가려지나 | 장착 상태 추력 비교, 간단한 유동 시험 | actuator disk 기반 3D 간섭 해석부터 유용 |
| 평판이 어떤 후류를 받고, 고받음각에서 어떤 모멘트가 생기나 | 완성 기체 힘/모멘트 측정 | 3D 정상·비정상 유동, 설계 비교 |
| 동체·붐의 측력, 양력, 압력 중심 | 전체 기체 6축 하중과 받음각·옆미끄럼 sweep | 동체 단독 → 날개 결합 → rotor disk 순서로 3D 하중 지도 |
| 로터 후류, 블레이드 통과, 급하강, 동적 실속 | 해당 현상을 노린 시험 | 회전 블레이드·비정상 CFD. 계산 비용과 검증 부담이 큼 |

CFD를 시작한다면 CAD, CG, 프로펠러 직경·피치, RPM, 추력, 회전 방향, 붙임각을 먼저 확정합니다. 기체만 →
로터 disk 포함 → 필요하면 비정상 회전 모델 순으로 비교하고, 격자·시간 간격 수렴, 경계조건과 난류·천이
모델의 영향, 측정값과의 차이를 기록합니다. 결과는 SITL에 solver를 넣는 대신 CL/CD/Cm과 장착 추력 보정
지도, 그리고 오차 범위로 옮기는 것이 맞습니다.

참고: NASA의 [multirotor CFD 연구](https://nas.nasa.gov/SC20/demos/demo7.html)는 disk 모델과 실제 회전
모델을 구분하고 실험으로 대조했습니다. [CFD 검증 안내](https://www.grc.nasa.gov/www/wind/valid/tutorial/overview.html)는
수치 해가 맞는지(verification)와 실제 물리와 맞는지(validation)를 구분합니다.
