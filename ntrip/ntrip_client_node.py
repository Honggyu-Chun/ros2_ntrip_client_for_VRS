import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message
import socket
import threading

class RTCMTCPBridgeNode(Node):
    def __init__(self):
        super().__init__('rtcm_tcp_bridge_node')

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2102)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Message, '/ntrip_client/rtcm', 10)
        self.recv_buffer = b''  # 수신 버퍼 초기화

        threading.Thread(target=self.tcp_listener, daemon=True).start()

    def tcp_listener(self):
        self.get_logger().info(f'🔌 TCP 연결 중: {self.host}:{self.port}')
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.host, self.port))
        self.get_logger().info(f'✅ 연결 성공. RTCM 데이터 수신 중...')

        while True:
            data = sock.recv(1024)
            if not data:
                self.get_logger().warn('⚠️ 연결 종료됨')
                break

            self.recv_buffer += data
            messages = self.extract_rtcm_messages()
            for msg_bytes in messages:
                msg = Message()
                msg.message = msg_bytes
                self.publisher_.publish(msg)
                self.get_logger().debug(f'📤 퍼블리시: {msg_bytes.hex()[:40]}... ({len(msg_bytes)} bytes)')

    def extract_rtcm_messages(self):
        messages = []
        i = 0
        buffer = self.recv_buffer

        while i < len(buffer) - 5:  # 최소 메시지 길이 확보
            if buffer[i] != 0xD3:
                i += 1
                continue

            if i + 3 > len(buffer):
                break  # 길이 필드까지 못 읽음

            # RTCM 길이 필드: 10비트
            length = ((buffer[i+1] & 0x03) << 8) | buffer[i+2]
            total_len = 3 + length + 3  # 헤더(3) + 페이로드 + CRC(3)

            if i + total_len > len(buffer):
                break  # 메시지 전부 도착 안함

            msg = buffer[i:i+total_len]
            messages.append(msg)
            i += total_len

        self.recv_buffer = buffer[i:]  # 사용한 부분 제거
        return messages

def main(args=None):
    rclpy.init(args=args)
    node = RTCMTCPBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
