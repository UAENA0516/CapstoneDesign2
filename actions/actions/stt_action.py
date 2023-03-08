import pyaudio
import speech_recognition as sr
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import threading

audio = pyaudio.PyAudio()

class Send_action(Node):
    def __init__(self):
        super().__init__('send_action')
        qos_profile = QoSProfile(depth=10)
        self.action_publisher = self.create_publisher(String, 'action_command', qos_profile)
        
        for index in range(audio.get_device_count()):
            desc = audio.get_device_info_by_index(index)
            print("DEVICE: {device}, INDEX: {index}, RATE: {rate} ".format(
                device=desc["name"], index=index, rate=int(desc["defaultSampleRate"])))

        
              
        th = threading.Thread(target=self.key_input)
        th.start()
        
    def key_input(self):
        r = sr.Recognizer()
        while True:
            with sr.Microphone() as source:
                print('동작중')
                audio = r.listen(source)
            try:
                stt_text = r.recognize_google(audio, language='ko')
            except sr.UnknownValueError:
                print("동작 실패")
            except sr.RequestError as e:
                print('요청 실패 : {0}'.format(e))  

            filtered_text = self.text_filter(stt_text)

            msg = String()
            msg.data = filtered_text
            self.action_publisher.publish(msg)

    def text_filter(self, text):
        
        if '앉아' in text or '안자' in text:
            text = '앉아'
        elif '빵' in text:
            text = '빵'
        elif '인사' in text:
            text = '인사'
        elif '일어서' in text:
            text = '일어서'

        return text

def main(args=None):
    rclpy.init(args=args)
    node = Send_action()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

