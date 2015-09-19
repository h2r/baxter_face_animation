#!/usr/bin/env python

import glob
import copy
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32, String
import rospkg


class Hear_orders:
    def __init__(self):
        self.speech_subscriber = rospy.Subscriber("/speech_recognition", String, self.publish_emotion)
        self.emotion_publisher = rospy.Publisher("/emotion", String, queue_size=10)

        # self.timer = rospy.Timer(rospy.Duration(self.velocity), self.timer_cb)

    def publish_emotion(self, data):
        self.emotion_publisher.publish("heard_an_order")

 


   
    

        



def main():
    rospy.init_node('hearing_node', anonymous=True)
    rate = rospy.Rate(30)
    rospack = rospkg.RosPack()
    # path = rospack.get_path('baxter_face_animation') + "/data/"

    Hear_orders()
    while not rospy.is_shutdown():
        rate.sleep()
    

if __name__ == "__main__":
    main()
