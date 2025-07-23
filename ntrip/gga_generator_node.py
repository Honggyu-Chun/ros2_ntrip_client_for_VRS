import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import socket
import threading
import time
import base64
import os

class NTRIPClientNode(Node):
    def __init__(self):
        super().__init__('ntrip_client_node')

        # 파라미터로부터 NTRIP 서버 정보 받기
        # 사용자 이름, 비밀번호, 서버 주소, 포트, 마운트포인트를 파라미터로 받는다.
        self.username = self.declare_parameter('username', 'honggyu03').get_parameter_value().string_value
        self.password = self.declare_parameter('password', 'ngii').get_parameter_value().string_value
        self.server_ip = self.declare_parameter('server_ip', 'RTS1.ngii.go.kr').get_parameter_value().string_value
        self.server_port = self.declare_parameter('server_port', 2101).get_parameter_value().integer_value
        self.mountpoint = self.declare_parameter('mountpoint', 'VRS-RTCM31').get_parameter_value().string_value
        
        self.client_socket = None
        self.reconnect_attempts = 0  # 재연결 시도 횟수
        self.max_reconnect_attempts = 5  # 최대 재연결 시도 횟수
        self.gga_sent = False  # GGA 송신 여부 추적
        self.rtcm_received = False  # RTCM 수신 여부 추적
        self.rtcm_msg = ""  # RTCM 메시지
        self.rtcm_count = 0  # RTCM 메시지 퍼블리시 횟수

        self.connect_to_ntrip_server()  # 서버 연결 함수 호출

        # 위치 정보 수신
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            10
        )

        # RTCM 데이터를 퍼블리시할 토픽을 생성
        self.rtcm_publisher = self.create_publisher(String, '/ntrip_client/rtcm', 10)

        # RTCM 데이터 버퍼 초기화
        self.rtcm_buffer = b''

    def connect_to_ntrip_server(self):
        """NTRIP 서버에 연결 및 인증 헤더 송신"""
        # NTRIP 서버에 연결하기 위해 소켓 생성
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            # NTRIP 서버에 연결 시도
            self.client_socket.connect((self.server_ip, self.server_port))
            self.get_logger().info(f"📡 NTRIP 서버에 연결됨: {self.server_ip}:{self.server_port}")
            
            # 연결 후 인증 요청 보내기
            self.send_ntrip_request()
            
        except Exception as e:
            self.get_logger().error(f"❌ NTRIP 서버 연결 실패: {e}")
            # 연결 실패 시 재연결 시도
            self.reconnect_attempts += 1
            if self.reconnect_attempts <= self.max_reconnect_attempts:
                self.get_logger().warn(f"⚠️ 재연결 시도 {self.reconnect_attempts}/{self.max_reconnect_attempts}...")
                time.sleep(5)  # 재연결 대기 시간
                self.connect_to_ntrip_server()  # 재연결 시도
            else:
                self.get_logger().error(f"❌ 최대 재연결 시도 횟수 초과")
                return

    def send_ntrip_request(self):
        """NTRIP 클라이언트 요청 메시지 송신 (헤더 포함)"""
        # NTRIP 서버에 요청 메시지 전송 (헤더 포함)
        request = f"GET /{self.mountpoint} HTTP/1.1\r\n"  # 마운트포인트 추가
        request += f"Authorization: Basic {self.get_authentication_string()}\r\n"  # Base64로 인코딩된 인증 정보
        request += f"User-Agent: NTRIP Client\r\n"
        request += f"Host: {self.server_ip}:{self.server_port}\r\n"
        request += "Connection: close\r\n\r\n"

        # 서버로 요청 전송
        self.client_socket.sendall(request.encode())
        self.get_logger().info(f"📤 NTRIP 요청 전송: {request[:50]}...")

    def get_authentication_string(self):
        """사용자 이름과 비밀번호를 Base64로 인코딩하여 인증 문자열 생성"""
        # 인증 문자열 생성 (username:password 형식으로)
        auth_string = f"{self.username}:{self.password}"
        return base64.b64encode(auth_string.encode()).decode()

    def fix_callback(self, msg):
        """위치 정보 수신 후 GGA 메시지 생성 및 송신"""
        # 위치 정보가 유효하고 연결된 소켓이 있으면 GGA 메시지를 생성하여 서버로 송신
        if msg.status.status < 0 or self.client_socket is None:
            return

        # GGA 메시지 생성
        gga = self.generate_gga(msg.latitude, msg.longitude, msg.altitude)
        try:
            # GGA 메시지를 NTRIP 서버로 송신
            self.client_socket.sendall((gga + '\r\n').encode())
            self.get_logger().info(f"📤 GGA 메시지 송신: {gga[:40]}...")
            self.gga_sent = True
            
            # GGA 송신 후 RTCM 수신을 위한 추가 코드 삽입
            self.receive_rtcm()

        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().warn("⚠️ NTRIP 서버 연결 끊김. 다시 연결 중...")
            self.client_socket.close()
            self.connect_to_ntrip_server()

    def generate_gga(self, lat, lon, alt):
        """위도, 경도, 고도를 받아 GGA 메시지 생성"""
        # 위도, 경도 값을 '도.분' 형식으로 변환
        def to_deg_min(val, is_lat):
            deg = int(abs(val))
            minutes = (abs(val) - deg) * 60
            hemi = 'N' if val >= 0 else 'S' if is_lat else 'E' if val >= 0 else 'W'
            return (f'{deg:02d}' if is_lat else f'{deg:03d}') + f'{minutes:07.4f}', hemi

        # GGA 메시지 생성
        lat_str, lat_hemi = to_deg_min(lat, True)
        lon_str, lon_hemi = to_deg_min(lon, False)
        timestamp = time.strftime("%H%M%S.00", time.gmtime())

        gga = f"$GPGGA,{timestamp},{lat_str},{lat_hemi},{lon_str},{lon_hemi},1,12,1.0,{alt:.1f},M,0.0,M,,"
        checksum = 0
        for c in gga[1:]:
            checksum ^= ord(c)
        return gga + f"*{checksum:02X}"

    def receive_rtcm(self):
        """서버로부터 RTCM 메시지 수신 (패킷이 여러 개로 나눠서 들어올 경우 처리)"""
        try:
            # RTCM 수신 대기
            rtcm_data = self.client_socket.recv(1024)
            if rtcm_data:
                self.get_logger().info(f"📥 RTCM 데이터 수신: {rtcm_data[:40]}...")
                
                # 수신한 데이터를 버퍼에 저장
                self.rtcm_buffer += rtcm_data
                self.rtcm_received = True

                # RTCM 메시지가 완전한지 확인
                while len(self.rtcm_buffer) > 3:
                    if self.rtcm_buffer[0] == 0xD3:  # 패킷 시작 점검
                        packet_length = (self.rtcm_buffer[1] & 0x03) << 8 | self.rtcm_buffer[2]
                        total_len = packet_length + 3 + 3  # 헤더 + 데이터 + CRC
                        
                        if len(self.rtcm_buffer) >= total_len:
                            # RTCM 메시지 완전 수신
                            rtcm_msg = self.rtcm_buffer[:total_len]
                            self.rtcm_buffer = self.rtcm_buffer[total_len:]
                            
                            # RTCM 메시지 퍼블리시
                            rtcm_msg_hex = rtcm_msg.hex()
                            self.rtcm_msg = rtcm_msg_hex  # RTCM 메시지 저장
                            self.rtcm_count += 1  # RTCM 메시지 퍼블리시 횟수 증가
                            rtcm_message = String()
                            rtcm_message.data = rtcm_msg_hex
                            self.rtcm_publisher.publish(rtcm_message)
                            self.get_logger().info(f"📤 RTCM 메시지 퍼블리시: {rtcm_msg_hex[:40]}...")
                        else:
                            break
                    else:
                        # 패킷 시작이 아니면 버퍼에서 하나씩 빼서 확인
                        self.rtcm_buffer = self.rtcm_buffer[1:]

        except (socket.timeout, socket.error) as e:
            self.get_logger().warn(f"⚠️ RTCM 데이터 수신 오류: {e}")

    def display_status(self):
        """실시간으로 상태를 테이블로 출력"""
        os.system('clear')  # 화면 초기화
        print("📊 실시간 NTRIP 상태")
        print("────────────────────────────────────")
        print(f"GGA 송신 여부: {self.gga_sent}")
        print(f"서버 응답: {self.client_socket is not None}")
        print(f"RTCM 수신 여부: {self.rtcm_received}")
        print(f"RTCM 메시지: {self.rtcm_msg[:40]}...")
        print(f"RTCM 메시지 퍼블리시 횟수: {self.rtcm_count}")
        print("────────────────────────────────────")

    def destroy_node(self):
        """노드 종료 시 소켓 연결 종료"""
        if self.client_socket:
            self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NTRIPClientNode()
    try:
        while rclpy.ok():
            node.display_status()  # 상태를 실시간으로 출력
            rclpy.spin_once(node)  # 콜백 처리
    finally:
        node.destroy_node()
        rclpy.shutdown()
