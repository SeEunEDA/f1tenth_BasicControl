# f1tenth_BasicControl

F1TENTH 시뮬레이터에서 **키보드 텔레옵 + Twist→Ackermann 변환 + PID 속도제어**를 한 번에 구동하는 과제 1용 베이스 레포.

- 시뮬: [`f1tenth_gym_ros`](https://github.com/f1tenth/f1tenth_gym_ros)
- 제어 패키지: `twist_to_ackermann_pid` ( `/cmd_vel` → `/drive(Ackermann)` + PID + 한계/필터 )

---

## 요구사항

- Ubuntu 20.04 + **ROS 2 Foxy**
- 패키지 의존
  ```bash
  sudo apt update
  sudo apt install -y \
    ros-foxy-ackermann-msgs \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-rqt ros-foxy-rqt-common-plugins ros-foxy-rqt-plot

설치 & 빌드

# 워크스페이스
mkdir -p ~/sim_ws/src && cd ~/sim_ws/src

# 시뮬레이터
git clone https://github.com/f1tenth/f1tenth_gym_ros.git

# 본 레포
git clone git@github.com:SeEunEDA/f1tenth_BasicControl.git .

# 빌드 & 환경설정
cd ~/sim_ws
colcon build --symlink-install
echo 'source /opt/ros/foxy/setup.bash'  >> ~/.bashrc
echo 'source ~/sim_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

패키지 구조

twist_to_ackermann_pid/
  ├─ package.xml
  ├─ setup.py
  ├─ setup.cfg
  ├─ resource/twist_to_ackermann_pid
  ├─ twist_to_ackermann_pid/
  │   ├─ __init__.py
  │   └─ twist_to_ackermann_pid.py       # 노드: /cmd_vel → /drive, PID 포함
  ├─ config/
  │   └─ twist_to_ackermann_pid.yaml     # 파라미터
  └─ launch/
      ├─ twist_to_ackermann_pid.launch.py
      └─ basic_control_bringup.launch.py # 시뮬 + PID 통합 런치

실행(Quick Start)
터미널 1 — 시뮬 + PID 변환

ros2 launch twist_to_ackermann_pid basic_control_bringup.launch.py

터미널 2 — 키보드 텔레옵

ros2 run teleop_twist_keyboard teleop_twist_keyboard

키: i(전진), ,(후진), u/o(전진+좌/우), m/.(후진+좌/우), k(정지)

    중요: 텔레옵 창에 포커스를 둬야 입력이 전달됩니다.

모니터링

    토픽 목록

ros2 topic list | egrep "/(cmd_vel|drive|cmd_v|meas_v|pid_error)"

rqt_plot 그래프

ros2 run rqt_plot rqt_plot \
  /cmd_v/data /meas_v/data /pid_error/data \
  /drive/drive/speed /drive/drive/steering_angle

그래프에서 Home → Autoscale을 누르면 보기 편합니다.

Foxy에서 잠깐 값 확인

    timeout 1 ros2 topic echo /drive
    ros2 topic hz /drive

주요 파라미터 (config/twist_to_ackermann_pid.yaml)

wheelbase: 0.33                 # 차축간 거리 [m]
max_speed: 2.5                  # 최대 속도 [m/s]
max_steering_deg: 28.0          # 최대 조향각 [deg]
rate_hz: 50.0                   # 제어 루프 주기 [Hz]
accel_limit: 2.0                # 가속도 제한 [m/s^2]
max_steer_rate_deg_s: 240.0     # 조향각 변화 제한 [deg/s]

kp: 0.8
ki: 0.2
kd: 0.04
i_limit: 2.0
vel_lpf_alpha: 0.2              # 속도 저역통과 필터(작을수록 더 부드러움)
stop_deadband: 0.05             # 거의 정지 취급 구간 [m/s]

런타임 튜닝 예시

ros2 param set /twist_to_ackermann_pid kp 0.6
ros2 param set /twist_to_ackermann_pid kd 0.08
ros2 param set /twist_to_ackermann_pid ki 0.0
ros2 param set /twist_to_ackermann_pid accel_limit 1.2
ros2 param set /twist_to_ackermann_pid max_steer_rate_deg_s 150

튜닝 가이드(증상별)
증상	조치
가속 시 확 튐/떨림	kp↓, kd↑, accel_limit↓
목표속도 도달이 느림	kp↑, accel_limit↑
속도 진동/오버슈트	kd↑, kp↓, vel_lpf_alpha↓(0.1~0.2)
정지에서 꿈틀거림	stop_deadband↑(0.07~0.1), ki↓ 또는 0
직진 지그재그	max_steer_rate_deg_s↓(120~180), max_steering_deg↓(25~28)
좌/우 반대	코드에서 delta = -delta 부호 반전 필요
문제 해결(Quick Checklist)

    /drive가 안 나옴 → 브링업 실행 여부, 노드 에러, 파라미터 로드 확인.

    rqt_plot이 빈 화면 → 토픽 경로를 필드까지 넣기 (/cmd_v/data, /drive/drive/speed), Autoscale, 텔레옵 창 포커스 확인.

    텔레옵 각속도 0 → u/o 또는 j/l 키 사용 중인지 확인.

라이선스

MIT
