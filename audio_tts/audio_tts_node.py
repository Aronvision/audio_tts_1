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

        self.get_logger().info('Audio TTS Node가 시작되었습니다.')

        try:
            # TTS 모델 초기화
            self.custom_tts = Custom_TTS()
            self.custom_tts.set_model(language='KR')

            # 레퍼런스 스피커 설정 (예시로 'reference_speaker.wav' 파일 사용)
            reference_speaker_path = os.path.join(os.path.dirname(__file__), 'sample_sunhi.mp3')
            if not os.path.exists(reference_speaker_path):
                self.get_logger().error(f'레퍼런스 스피커 파일이 존재하지 않습니다: {reference_speaker_path}')
                rclpy.shutdown()
                return
            self.custom_tts.get_reference_speaker(reference_speaker_path)
        except Exception as e:
            self.get_logger().error(f'초기화 중 오류 발생: {e}')
            rclpy.shutdown()
            return

    def listener_callback(self, msg):
        chatgpt_reply = msg.data
        self.get_logger().info(f'ChatGPT 응답 수신: "{chatgpt_reply}"')

        # TTS 수행
        try:
            self.custom_tts.make_speech(chatgpt_reply)
            self.get_logger().info('TTS 변환 및 음성 변조 완료')
            # 음성 파일 재생 (필요한 경우)
            output_audio_path = f'{self.custom_tts.output_path}/result_{self.custom_tts.result_cnt - 1}.wav'
            os.system(f'play {output_audio_path}')  # 'sox' 패키지가 설치되어 있어야 합니다.
        except Exception as e:
            self.get_logger().error(f'TTS 처리 중 오류 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AudioTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
