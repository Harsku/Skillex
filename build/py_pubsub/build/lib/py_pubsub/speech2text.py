# https://www.thepythoncode.com/article/using-speech-recognition-to-convert-speech-to-text-python

import speech_recognition as sr 
import rclpy
from pyaudio import PyAudio
from rclpy.node import Node
from turtlesim.msg import Pose


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Pose, 'turtlesim', 10)
        self.msg = Pose()
        self.text = ""
        r = sr.Recognizer()
        m = sr.Microphone()
        with m as source:
            r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening

        # start listening in the background (note that we don't have to do this inside a `with` statement)
        self.stop_listening = r.listen_in_background(m, self.callback)

    def __del__(self):
        self.stop_listening(wait_for_stop=False)

    # this is called from the background thread
    def callback(self, recognizer, audio):
        # received audio data, now we'll recognize it using Google Speech Recognition
        try:
            # for testing purposes, we're just using the default API key
            # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            self.text = recognizer.recognize_google(audio)
            print("Google Speech Recognition thinks you said " + self.text)
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        
        if self.compile_msg():
            self.publisher_.publish(self.msg)


    def compile_msg(self):

        if "up" in self.text:
            self.msg.x = float(0)
            self.msg.y = float(-10)
        elif "down" in self.text:
            self.msg.x = float(0)
            self.msg.y = float(0)
        elif "left" in self.text:
            self.msg.x = float(-10)
            self.msg.y = float(0)
        elif "right" in self.text:
            self.msg.x = float(10)
            self.msg.y = float(0)
        else:
            return False
        return True

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()