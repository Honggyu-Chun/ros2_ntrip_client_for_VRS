ros2_ntrip_client_for_VRS

이 프로젝트는 ROS 2 환경에서 NTRIP 클라이언트를 구현하여, RTK-GPS 데이터를 실시간으로 수신하고 처리하는 시스템입니다. 이 시스템은 주로 GNSS 기반의 위치 정확도를 개선하는 RTK (Real-Time Kinematic) 데이터를 활용하는 로봇 및 자율주행 시스템에 사용될 수 있습니다.
주요 기능

    gga_generator_node.py: NTRIP 클라이언트로서, GGA 메시지를 주기적으로 생성하고 이를 전송하여 RTK 서비스를 요청합니다.

    rtcm_decoder_node.py: gga_generator_node.py가 퍼블리시한 /ntrip_client/rtcm 메시지를 구독하여, 수신된 RTCM 메시지의 종류를 실시간으로 디코딩하고 이를 사용자에게 알립니다.

설치 방법
필수 의존성

    ROS 2 (Humble 또는 그 이상)

    rtcm_msgs ROS 2 패키지

    rclpy ROS 2 Python 라이브러리

    Python 3.x

설치 단계

    ROS 2 환경 설치
    ROS 2 Humble 이상을 설치해 주세요. ROS 2 설치 방법은 ROS 2 공식 사이트를 참조해 주세요.

    패키지 클론
    프로젝트를 ROS 2 워크스페이스에 클론합니다.

cd ~/ros2_ws/src
git clone https://github.com/your-username/ros2_ntrip_client_for_VRS.git

의존성 설치
패키지 의존성을 설치합니다.

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

빌드
패키지를 빌드합니다.

colcon build --symlink-install

환경 설정
ROS 2 환경을 설정합니다.

    source install/setup.bash

사용 방법
1. gga_generator_node.py 실행

gga_generator_node.py는 NTRIP 클라이언트 역할을 하며, GNSS 데이터를 실시간으로 요청하고 GGA 메시지를 퍼블리시합니다.

실행 방법:

ros2 run ntrip gga_generator_node

2. rtcm_decoder_node.py 실행

rtcm_decoder_node.py는 /ntrip_client/rtcm 메시지를 구독하여 수신된 RTCM 메시지의 종류를 디코딩하고 이를 출력합니다.

실행 방법:

ros2 run ntrip rtcm_decoder_node

3. NTRIP 클라이언트 설정

gga_generator_node.py는 다음 파라미터를 사용하여 NTRIP 서버와 연결합니다.

    host: NTRIP 서버 주소 (기본값: localhost)

    port: NTRIP 서버 포트 (기본값: 2101)

    mountpoint: 사용할 마운트 포인트 (기본값: VRS-RTCM31)

    username, password: 인증 정보 (기본값: 없음)

이 파라미터들은 ros2 param 명령어를 사용하여 실시간으로 변경할 수 있습니다.
예시 출력

rtcm_decoder_node.py 실행 시, 다음과 같은 출력이 나타날 수 있습니다.

[INFO] [rtcm_decoder_node]: RTCM 메시지 타입 1004 수신
[INFO] [rtcm_decoder_node]: RTCM 메시지 타입 1012 수신
[INFO] [rtcm_decoder_node]: RTCM 메시지 타입 1033 수신

참고

이 프로젝트는 RTCM 3.1 메시지 포맷을 사용하며, GGA 메시지를 주기적으로 생성하고 이를 NTRIP 클라이언트에 전달하여 RTK 데이터를 요청합니다. 각 RTCM 메시지의 타입은 rtcm_decoder_node에서 구독하여 디코딩됩니다.
