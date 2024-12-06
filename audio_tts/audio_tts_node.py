# audio_tts/audio_tts_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .custom_tts import Custom_TTS  # 현재 패키지에서 import
import os

class AudioTTSNode(Node):
    def __init__(self):
        super().__init__('audio_tts_node')

        # ChatGPT 응답을 구독
        self.subscription = self.create_subscription(
            String,
            'chatgpt_response',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # route_guidance_done 토픽 구독
        self.route_guidance_sub = self.create_subscription(
            String,
            'route_guidance_done',
            self.route_guidance_callback,
            10)
        
        # route_completed 토픽 발행
        self.route_done_publisher = self.create_publisher(
            String,
            'route_completed',
            10)

        self.get_logger().info('Audio TTS Node가 시작되었습니다.')

        try:
            # TTS 모델 초기화
            self.custom_tts = Custom_TTS()
            self.custom_tts.set_model(language='ko')  # 'KR'에서 'ko'로 변경
        except Exception as e:
            self.get_logger().error(f'초기화 중 오류 발생: {e}')
            rclpy.shutdown()
            return

    def listener_callback(self, msg):
        chatgpt_reply = msg.data
        self.get_logger().info(f'ChatGPT 응답 수신: "{chatgpt_reply}"')

        # TTS 수행
        try:
            output_path = self.custom_tts.make_speech(chatgpt_reply)
            if output_path:
                self.get_logger().info('TTS 변환 완료')
                os.system(f'mpg321 {output_path}')  # play 대신 mpg321 사용
        except Exception as e:
            self.get_logger().error(f'TTS 처리 중 오류 발생: {e}')

    def route_guidance_callback(self, msg):
        location = msg.data
        self.get_logger().info(f'길 안내 완료: "{location}"')

        # 위치 매핑 딕셔너리
        location_mapping = {
            '응급실': '0',
            '수납': '1',
            '접수': '1',
            '편의점': '2',
            '화장실': '3'
        }

        try:
            guidance_message = f'{location}로 안내를 시작합니다.'
            output_path = self.custom_tts.make_speech(guidance_message)
            if output_path:
                self.get_logger().info('길 안내 TTS 변환 완료')
                os.system(f'mpg321 {output_path}')

            # route_completed 토픽으로 매핑된 숫자값 퍼블리시
            route_done_msg = String()
            route_done_msg.data = location_mapping.get(location, '0')  # 매핑되지 않은 경우 기본값 '0'
            self.route_done_publisher.publish(route_done_msg)
            self.get_logger().info(f'route_completed 토픽에 "{route_done_msg.data}" 메시지를 퍼블리시했습니다.')
        except Exception as e:
            self.get_logger().error(f'길 안내 TTS 처리 중 오류 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AudioTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
