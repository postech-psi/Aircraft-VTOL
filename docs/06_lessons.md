# 6. 교훈

시뮬레이션과 코드 검토로 배운 것과, 같은 구성을 다룬 연구·PX4·ArduPilot 문서에서 확인한 것 중 연말 비행의
결정에 직접 걸리는 것만 모았습니다. 항목마다 무엇을 배웠는지, 그래서 무엇을 하는지(→) 순서로 적었습니다.
시뮬레이션 수치의 근거는 [보고서](../missions/2026_year_end/report.md)에 있습니다.

## 기체

**1. yaw가 가장 약한 축이고, 게인으로는 늘지 않는다.** 조종면이 없으므로 yaw는 리프트 로터의 반토크 차로만
냅니다([2. 비행 방법](02_flight.md#6-날개-양력과-제어-여유)). SITL에서 8 m/s 측풍의 기수 오차는 18–22°로
기준(20°) 근처였고, 게인을 바꿔도 나아지지 않았습니다. ArduPilot도 quadplane은 멀티콥터만큼 yaw 권한을 갖는
일이 드물고, 모터가 조금 비틀려 달리기만 해도 호버의 yaw를 잃을 수 있으며, 모터를 일부러 2–3° 기울여 달면
수직 추력을 조금만 잃고 yaw 권한이 크게 는다고 안내합니다[1]. 로터를 기울이면 외란 속에서 전력도 준다는
실험이 있습니다[2]. → 실기체에서 yaw 권한을 먼저 재고 모터 장착 각도를 확인합니다. 모자라면 모터 기울임을 첫
기체 변경으로 검토합니다.

**2. 푸셔는 정지 추력이 아니라 비행 중 추력으로 정한다.** 프로펠러 추력은 유입 속도가 커질수록 줄어듭니다.
quadplane 추진기의 풍동 시험에서 15 m/s의 최대 추력은 정지 추력의 44%였습니다[3]. SITL에서도 10 m/s 순항에
6–8 m/s 정풍이 겹치면 푸셔가 포화됐고, 8 m/s 정풍에서는 목표 속도에 닿지 못했습니다. → 푸셔는 대기속도별 추력을 잽니다. 잴 수
없으면 같은 계열 프로펠러의 풍동 자료[4]로 추정합니다. 순항은 5 m/s로 둡니다.

**3. 바람 속에서는 기체를 어느 쪽으로 세우느냐가 결과를 가른다.** quadplane은 VTOL 모드에서 날개 때문에 바람에
약하고, CG 위치와 로터 기울기가 바람 저항을 크게 바꿉니다[5]. 기수를 바람에 맞추면 호버 정확도와 바람 저항이
좋아집니다[6]. SITL에서 6 m/s(돌풍 30%)까지는 네 방향 모두 통과했지만 8 m/s 배풍 이륙은 pitch 35–45°로
실패했습니다. → 바람을 마주 보고 이륙하고, 착륙 전에는 기수를 바람 쪽으로 돌립니다(구현됨). 평균 6 m/s를
넘으면 날지 않습니다.

**4. 리프트 모터 하나를 잃으면 이 기체는 자세를 지킬 수 없다.** 모터 네 개짜리 기체는 하나를 잃으면 yaw를
포기하고 기체를 계속 돌리는 특별한 제어로만 버틸 수 있습니다[7]. stock PX4에는 그런 동작이 없고, SITL에서도
주입하지 않았습니다. 소프트웨어로 줄일 수 없는 위험입니다. → 비행마다 모터·ESC·프로펠러를 점검하고, 사람과
재산에서 100 m 넘게 떨어져 날고, 조종자가 바로 쓸 수 있는 비상 스위치를 둡니다[8].

## PX4와 탑재 소프트웨어

**5. PX4 기본값 중에는 이 기체에 위험한 것이 있다.** 기본값으로 SITL에서 날렸을 때 Jetson이 멈춘 뒤 15초
넘게 5 m/s로 직진했고(`COM_OBL_RC_ACT`), PX4의 제동에 25° 기울었고(`MPC_ACC_HOR`), 천이 명령에 roll 43°까지
기울었습니다. 기본 귀환은 60 m까지 올라간 뒤 돌아오고, 배터리 critical에는 경고만 합니다. → 바꾼
값을 모두 이유와 함께 `px4/params.yaml`에 두고, 시뮬레이션과 실기체에 같은 파일에서 만든 값만 씁니다
([3. PX4 설정](03_px4_setup.md#4-파라미터)).

**6. 보내는 명령은 "이것이 마지막 명령이 되어도 안전한가"로 고른다.** PX4는 setpoint가 끊겨도
`COM_OF_LOSS_T` 동안 마지막 setpoint를 따릅니다[9]. 거기에 이미 지나친 위치가 있으면 그 점으로 급제동합니다
(SITL 29°). → 움직이는 동안에는 위치 대신 속도를 보냅니다(12°로 멈춤).

**7. 고장 뒤의 모드는 그 모드가 그 고장에서 무엇을 하는지 보고 고른다.** 푸셔가 죽은 기체에 Hold는 안전하지
않았습니다. 푸셔 보조가 켜진 Hold에서 PX4는 죽은 푸셔를 최대로 돌리며 명령을 쌓았고, Land로 바꾸자 ±45°로
흔들렸습니다. 천이를 시도하는 동안에는 offboard 명령을 따르지 않습니다. → 고장마다 넘길 모드를 정해
두고(`profile.needs_land`), 그 조합을 SITL에서 비행해 확인합니다.

**8. 명령을 계단처럼 바꾸지 않는다.** PX4는 offboard setpoint를 거르지 않고 씁니다. 가속도가 계단처럼 바뀌자
기울기와 푸셔 명령도 계단처럼 바뀌어 감속 시작에 기수가 7.2° 튀었습니다. → 곡률이 연속인 경로[10]와 가속도
변화율을 제한한 속도 계획[11]을 씁니다([2. 비행 방법](02_flight.md#2-경로-계획)).

## 시뮬레이션

**9. 시뮬레이션은 조건을 비교하는 데 쓰고, 한계값은 잰 뒤 다시 정한다.** 기체 계수의 대부분이 가정값입니다.
값을 하나씩 바꿔 본 61가지 중 판정 여유를 가장 많이 쓴 것은 CG, 푸셔 추력, 장착 상태의 로터 효율이었습니다.
하나씩 바꾸는 시험은 값끼리의 상호작용을 놓칩니다[12]. → 이 셋을 먼저 재고, 잰 값으로 바람과 속도 한계를 다시
돌립니다. 결정에 쓰는 조합(정풍 + 푸셔 고장, 측풍 모서리 + 링크 상실)은 시나리오로 직접 비행합니다.

**10. 고장 주입이 정말 그 고장을 만드는지 먼저 확인한다.** PX4 v1.17의 모터 고장 주입은 SITL에서 ESC
텔레메트리만 지우고 모터는 계속 돌렸습니다[13]. 그대로 믿었다면 "푸셔가 고장 나도 문제없다"는 잘못된 결론을
냈을 것입니다. → 푸셔는 공력 플러그인에서 끄고, 사건마다 기대한 대응(`sweep.EXPECTED`)이 실제로 일어났는지
판정합니다.

**11. 시뮬레이션은 혼자 돌린다.** PX4의 시계는 Gazebo를 따르지만 Gazebo는 PX4의 모터 명령을 기다리지
않습니다. PX4 문서는 노트북에서 실제의 3–4배 정도가 나온다고 합니다[14]. 4 CPU에서 4배로 여러 개를 함께 돌리자
정사각형 11회 중 6회에 1 Hz 흔들림이 생겼고, 혼자 돌린 13회에는 없었습니다. → 한 번에 하나씩(`--jobs 1`) 돌리고,
함께 돌던 PX4 수를 결과에 남깁니다.

## 비행 시험

**12. 한 번에 하나씩 넓히고, 단계마다 빠져나올 모드를 정해 둔다.** ArduPilot은 quadplane을 각속도 게인 →
고도 유지 → 위치 유지 → 자동 비행 순서로 맞추라고 합니다[15]. PX4는 첫 비행에 경험자를 동반하고, 경로와 착륙
지점을 미리 정하고, 통제를 잃으면 빈 곳에 떨어뜨리라고 합니다[8]. → [비행 시험 순서](07_roadmap.md#2-비행-시험-순서)대로
단계마다 하나만 바꾸고, 빠져나올 모드와 SITL에서 나온 비교 기준을 정해 둡니다.

**13. 앞으로 날기 전에 호버 로그로 진동, 자력계, 배터리를 본다.** 진동이 크면 센서가 포화되고 위치 추정이
틀어집니다[16]. 전원선 가까운 자력계는 전류에 따라 기수를 틀리게 읽습니다[17]. 배터리 전압은 부하에 처지므로
전류 센서를 보정해야 잔량이 맞습니다[18]. eCalc의 호버 10.6분은 계산값이고, 시뮬레이션에는 배터리 모델이
없습니다. → 셋을 확인한 뒤에 전진 비행을 시작하고, 실제 비행 시간을 재서 착륙 여유를 정합니다.

**14. 조종 자격과 기체 신고는 무게로 정해진다.** 국내 조종자 증명은 최대이륙중량으로 나뉘어 2 kg 초과 7 kg
이하는 3종, 7 kg 초과 25 kg 이하는 2종이고, 비영리 기체도 2 kg을 넘으면 신고 대상입니다[19]. 이 기체는 7 kg으로
계산했으므로 무엇을 더 달면 2종이 됩니다. → 비행 상태 그대로 무게를 재고, 한국교통안전공단의 현행 기준으로
자격과 신고를 확인합니다.

## 참고 문헌

1. ArduPilot, [QuadPlane Setup Tips](https://ardupilot.org/plane/docs/quadplane-tips.html) — yaw 권한과 모터 기울임.
2. C. Holda, B. Ghalamchi, M. W. Mueller, "Tilting Multicopter Rotors for Increased Power Efficiency and Yaw
   Authority," *ICUAS*, 2018. [PDF](https://hiperlab.berkeley.edu/wp-content/uploads/2018/05/2018_TiltingMulticopterRotors.pdf)
3. A. Mathur, E. Atkins, "Wind Tunnel Testing and Aerodynamic Characterization of a QuadPlane Uncrewed Aircraft
   System," 2023. [arXiv:2301.12316](https://arxiv.org/abs/2301.12316)
4. J. B. Brandt, M. S. Selig, "Propeller Performance Data at Low Reynolds Numbers," AIAA 2011-1255.
   [PDF](https://m-selig.web.engr.illinois.edu/pubs/BrandtSelig-2011-AIAA-2011-1255-LRN-Propellers.pdf),
   [UIUC Propeller Database](https://m-selig.ae.illinois.edu/props/propDB.html)
5. Li 외, "Dynamic Simulation and Conceptual Layout Study on a Quad-Plane in VTOL Mode in Wind Disturbance
   Environment," *International Journal of Aerospace Engineering* 2022, 5867825.
   [doi:10.1155/2022/5867825](https://doi.org/10.1155/2022/5867825)
6. S. Wang, X. Chen, H. Wei, Q. Quan, "Heading Adjustment by Admittance Control for Lifting-Wing Quadcopters in
   Strong Winds," *IEEE Robotics and Automation Letters* 10(4), 3558–3565, 2025.
   [초록](https://research.buaa.edu.cn/en/publications/heading-adjustment-by-admittance-control-for-lifting-wing-quadcop/)
7. M. W. Mueller, R. D'Andrea, "Stability and Control of a Quadrocopter Despite the Complete Loss of One, Two, or
   Three Propellers," *IEEE ICRA*, 45–52, 2014.
8. PX4, [First Flight Guidelines](https://docs.px4.io/v1.17/en/flying/first_flight_guidelines).
9. PX4, [Offboard Mode](https://docs.px4.io/v1.17/en/flight_modes/offboard).
10. T. Fraichard, A. Scheuer, "From Reeds and Shepp's to Continuous-Curvature Paths," *IEEE Transactions on
    Robotics* 20(6), 1025–1035, 2004. [HAL](https://hal-univ-tlse3.archives-ouvertes.fr/INRIA/inria-00000009)
11. L. Berscheid, T. Kröger, "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States,"
    *Robotics: Science and Systems*, 2021. [doi:10.15607/RSS.2021.XVII.015](https://doi.org/10.15607/RSS.2021.XVII.015)
12. NATO RTO, [RTO-EN-SCI-176, 3장](https://www.sto.nato.int/publications/STO%20Educational%20Notes/RTO-EN-SCI-176/EN-SCI-176-03.pdf)
    — 하나씩 바꾸는 시험과 실험 설계.
13. PX4, [Failure Injection](https://docs.px4.io/main/en/debug/failure_injection).
14. PX4, [Gazebo Simulation](https://docs.px4.io/v1.17/en/sim_gazebo_gz/) — 배속과 PX4 시계.
15. ArduPilot, [QuadPlane VTOL Tuning Overview](https://ardupilot.org/plane/docs/quadplane-vtol-tuning.html).
16. PX4, [Log Analysis using Flight Review](https://docs.px4.io/v1.17/en/log/flight_review) — 진동.
17. PX4, [Compass Power Compensation](https://docs.px4.io/v1.15/en/advanced_config/compass_power_compensation).
18. PX4, [Battery Estimation Tuning](https://docs.px4.io/v1.17/en/config/battery).
19. 서울시, [드론 조종자 자격 안내](https://mediahub.seoul.go.kr/archives/2005610). 현행 기준은 한국교통안전공단에서
    확인합니다.
