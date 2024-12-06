import os
import shutil
from gtts import gTTS

class Custom_TTS:
    def __init__(self, output_path='output'):
        self.result_cnt = 0
        self.output_path = output_path

        # output 폴더 삭제 후 시작
        if os.path.exists(self.output_path):
            shutil.rmtree(self.output_path)

    def set_model(self, language='ko'):
        '''
        언어 설정
        language: 언어 코드 (ko: 한국어, en: 영어, ja: 일본어 등)
        '''
        self.language = language
        print('TTS 설정 완료')

    def make_speech(self, text, speed=1.0):
        '''
        텍스트를 입력하면 TTS를 수행하는 함수
        '''
        try:
            # 경로 설정, 폴더 생성
            output_path = f'{self.output_path}/result_{self.result_cnt}.mp3'
            os.makedirs(self.output_path, exist_ok=True)

            # TTS 수행
            tts = gTTS(text=text, lang=self.language)
            tts.save(output_path)
            print('TTS 생성 완료')

            self.result_cnt += 1
            return output_path
        except Exception as e:
            print(f"Error in TTS generation: {e}")
            return None

