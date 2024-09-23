#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class TTSpeech:
    def __init__(self):
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=1)

    def speak(self, msg):
        self.tts_pub.publish(msg)
