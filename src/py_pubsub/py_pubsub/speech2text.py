# https://www.thepythoncode.com/article/using-speech-recognition-to-convert-speech-to-text-python

import speech_recognition as sr 
import rclpy
from pyaudio import PyAudio
#import rospy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Create a publisher which can "talk" to Turtlesim and tell it to move
        #pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # Create a Twist message and add linear x and angular z values
        self.move_cmd = Twist()
        self.move_cmd.angular.z = 0.0
        self.text = ""
        self.speed = 2
        r = sr.Recognizer()
        m = sr.Microphone()
        with m as source:
            r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening

        # start listening in the background (note that we don't have to do this inside a `with` statement)
        self.stop_listening = r.listen_in_background(m, self.callback, phrase_time_limit=5)


    # this is called from the background thread
    def callback(self, recognizer, audio):
        # received audio data, now we'll recognize it using Google Speech Recognition
        try:
            # for testing purposes, we're just using the default API key
            # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            self.text = recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        
        if self.compile_msg():
            self.pub.publish(self.move_cmd)
        
        self.move_cmd.linear.x = float(0)
        self.move_cmd.linear.y = float(0)
        self.move_cmd.angular.z = float(0)
        self.text = ""


    def compile_msg(self):
        print(self.text)
        if "up" in self.text:
            self.move_cmd.linear.x = float(0)
            self.move_cmd.linear.y = float(self.speed)
        elif "down" in self.text:
            self.move_cmd.linear.x = float(0)
            self.move_cmd.linear.y = float(-self.speed)
        elif "left" in self.text:
            self.move_cmd.linear.x = float(-self.speed)
            self.move_cmd.linear.y = float(0)
        elif "right" in self.text:
            self.move_cmd.linear.x = float(self.speed)
            self.move_cmd.linear.y = float(0)
        elif "fast" in self.text or "quick" in self.text:
            self.speed += 2
        elif "slow" in self.text:
            self.speed -= 2
        elif "spin" in self.text:
            self.speed = 0
            self.move_cmd.angular.z = float(20)
            self.speed = 5
        elif "circles" in self.text:
            self.move_cmd.linear.x = float(self.speed)
            self.move_cmd.angular.z = float(10)
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