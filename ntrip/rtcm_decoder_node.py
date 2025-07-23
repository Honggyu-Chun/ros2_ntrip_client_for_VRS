import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import defaultdict
import struct
import time
import os

# RTCM 메시지의 시작 부분을 나타내는 헤더 값 (0xD3)
RTCM_PREAMBLE = b'\xD3'

# RTCM Type을 카운팅하기 위한 defaultdict 선언
rtcm_counter = defaultdict(int)

class RTCMDecoderNode(Node):
    def __init__(self):
        super().__init__('rtcm_decoder_node')

        # /ntrip_client/rtcm 토픽을 구독하여 RTCM 메시지를 받기 위한 subscription 생성
        self.subscription = self.create_subscription(
            String,  # 구독할 메시지 타입: String
            '/ntrip_client/rtcm',  # 구독할 토픽 이름
            self.rtcm_callback,  # 메시지가 수신되면 호출될 콜백 함수
            10  # 큐 사이즈: 동시에 처리할 수 있는 메시지 수
        )
        self.get_logger().info("📡 /ntrip_client/rtcm 토픽을 구독 중...")  # 구독 시작 로그

        self.buffer = b''  # RTCM 데이터를 받을 버퍼
        self.last_update = time.time()  # 마지막 업데이트 시간 (주기적으로 출력하기 위한 기준)
        self.last_raw_data = b''  # 가장 최근에 수신된 원시 RTCM 데이터 저장용

    def rtcm_callback(self, msg):
        """수신된 RTCM 메시지를 디코딩하는 콜백 함수"""
        rtcm_data = bytes.fromhex(msg.data)  # 수신된 메시지(hex 데이터)를 바이너리로 변환
        self.buffer += rtcm_data  # 수신된 데이터를 기존 버퍼에 추가
        print(f"📦 수신된 원시 데이터 (전체 바이트): {rtcm_data.hex()}")  # 수신된 원시 데이터 출력

        # 버퍼에서 RTCM 메시지를 처리
        while True:
            # RTCM 메시지를 파싱하여 메시지 타입과 메시지 길이를 받음
            msg_type, consumed = self.parse_rtcm_message(self.buffer)
            if consumed == 0:
                break  # 아직 메시지가 완전하지 않다면 반복 종료
            if msg_type is not None:
                rtcm_counter[msg_type] += 1  # 해당 RTCM 메시지 타입을 카운트
                self.last_raw_data = self.buffer[:consumed]  # 가장 최근 원시 데이터 업데이트
            self.buffer = self.buffer[consumed:]  # 처리된 데이터를 버퍼에서 제거

        # 1초마다 실시간 상태 출력
        if time.time() - self.last_update > 1.0:
            self.print_table()  # 상태 테이블 출력
            self.last_update = time.time()  # 마지막 업데이트 시간 갱신

    def parse_rtcm_message(self, buffer):
        """RTCM 메시지를 파싱하여 메시지 타입과 길이를 반환"""
        if len(buffer) < 6:  # RTCM 헤더가 6바이트보다 적으면 메시지가 불완전
            return None, 0

        if buffer[0] != 0xD3:  # RTCM 헤더가 '0xD3'이어야 함
            return None, 1  # 헤더 불일치 → 한 바이트씩 건너뛰며 재동기화

        # 10비트 길이를 구함 (RTCM 메시지의 길이 필드는 10비트로 구성)
        length = ((buffer[1] & 0x03) << 8) | buffer[2]
        full_len = 3 + length + 3  # 헤더(3바이트) + payload + CRC(3바이트)

        if len(buffer) < full_len:
            return None, 0  # 전체 메시지 수신 대기

        # 메시지의 페이로드 (데이터 부분)
        payload = buffer[3:3 + length]
        
        # 첫 12비트로 msg_type 추출 (MSB 기준 정렬)
        if len(payload) < 2:
            return None, full_len

        msg_type = ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF  # 메시지 타입 추출

        return msg_type, full_len  # 메시지 타입과 메시지 전체 길이 반환

    def print_table(self):
        """실시간으로 RTCM Type 수신 현황을 테이블 형식으로 출력"""
        os.system('clear')  # 화면을 깨끗하게 초기화 (Windows에서는 'cls' 사용)
        print("📊 실시간 RTCM Type 수신 현황")
        print("────────────────────────────")
        # RTCM 타입별로 수신된 메시지의 개수를 출력
        for msg_type in sorted(rtcm_counter):
            print(f"  Type {msg_type:<4}: {rtcm_counter[msg_type]:>5} 회")
        print("────────────────────────────")
        print("\n📦 가장 최근 수신된 원시 데이터 (RTCM3):")
        print(self.last_raw_data.hex())  # 가장 최근에 수신된 RTCM 원시 데이터를 출력

def main(args=None):
    """ROS2 실행 루프"""
    rclpy.init(args=args)  # ROS2 노드 초기화
    node = RTCMDecoderNode()  # RTCM 디코더 노드 생성
    try:
        rclpy.spin(node)  # 노드를 실행 상태로 유지
    finally:
        node.destroy_node()  # 노드 종료 시 리소스 해제
        rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()  # main() 함수 호출
