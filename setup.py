from setuptools import setup, find_packages

package_name = 'audio_tts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={'audio_tts': ['sample_sunhi.mp3']},
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # 필요한 패키지들 추가
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ChatGPT 응답을 음성으로 변환하는 ROS 2 노드',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_tts_node = audio_tts.audio_tts_node:main',
        ],
    },
)

