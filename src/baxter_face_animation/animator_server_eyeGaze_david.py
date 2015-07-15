#!/usr/bin/env python

import glob
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32
import rospkg
import math
import baxter_interface
from  geometry_msgs.msg import Pose
import tf
import numpy as np
import time

class Animation:
    def __init__(self, directory):
        self.fnames = [fname for fname in glob.glob("%s/*" % directory)]
        self.fnames.sort()
        self.images = [cv2.imread(path) for path in self.fnames]
        self.animation_timer = None
        self.current_value = 0
        self.current_idx = 0
        self.set_velocity(1/20.0)
        self.current_target = None
        self._head = baxter_interface.Head()
        self._head.set_pan(0.0)
        self.listener = tf.TransformListener();
        self.timer = rospy.Timer(rospy.Duration(self.velocity), self.timer_cb) 
        self.image_publisher = rospy.Publisher("/robot/xdisplay", Image, queue_size=10)
        self.valuey_publisher = rospy.Publisher("/eyeGaze/valuey/state", Int32, queue_size=10)
        self.targety_publisher = rospy.Publisher("/eyeGaze/targety/state", Int32, queue_size=10)
        self.valuex_subscriber = rospy.Subscriber("/eyeGaze/Point/command2", Pose, self.set_value, queue_size=1)
        self.set_valuey(49);

 
    def set_velocity(self, velocity):
        if isinstance(velocity, Float32):
            velocity = velocity.data
        self.velocity = velocity
        if (self.animation_timer != None):
            self.animation_timer.shutdown()


    def set_idx(self, idx):
        self.current_idx = idx;
        self.current_value = int((float(idx) / len(self.images)) * 100)
        self.checkrep()
        return self.publish


    def set_value(self, value):
        #print(value)
        if isinstance(value, Pose):
            print "setting value from topic"
        x = value.position.x-0.15 # 0.15 is offset from head
        y = value.position.y # head offset is essentially zero
        z = value.position.z - 0.67 # offset from head
        pheta = math.atan2(y,x)
        if pheta < 0: 
            pheta = max(pheta, -1 * math.pi/2)
        elif pheta > 0: 
            pheta = min(pheta, math.pi/2)

        print "Pheta " + str(pheta*180/math.pi)

        gd = math.sqrt(x*x+y*y) #ground (xy) distance from head to object
        self.set_valuex(pheta);
        #self.set_targety(-2.0)
        self.set_targety(math.atan2(z,gd))
        time.sleep(5)
	self.set_valuex(0)
        self.set_targety(-1.0) 
        #self._head.set_pan(0.0)

    def set_valuex(self, value):
        print "set_valuex", value
        if isinstance(value, Int32):
            print "setting value from topic"
        print "xval = " + str(value*180/math.pi) 
        self._head.set_pan(value, speed=5, timeout=20)

    def set_valuey(self, value):
        self.current_value = value
        self.current_idx = value;
        self.checkrep()
        return self.publish_image()

    def checkrep(self):
        assert 0 <= self.current_idx < len(self.images), self.current_idx

    def publish_image(self):
        print(self.current_idx)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.image, encoding="bgr8")
        self.image_publisher.publish(msg)
        return self.images[self.current_idx]
        

    def set_targety(self, target):
        target = int((target+2)*(49/4.0))
        self.current_target = target


    @property
    def image(self):
        return self.images[self.current_idx]

    def publish_state(self):
        self.valuey_publisher.publish(self.current_value)
        self.targety_publisher.publish(self.current_target)
       
    def timer_cb(self, time):
        self.animate()

    def animate(self):
        if self.current_target != None:
            print "target", self.current_target, self.current_value
            if self.current_target < self.current_value:
                self.set_valuey(self.current_value - 1)
            elif self.current_target > self.current_value:
                self.set_valuey(self.current_value + 1)
            elif self.current_target == self.current_value:
                self.current_target = None
            else:
                pass
                #raise ValueError("No target: " + `self.target`)

def main():
    rospy.init_node('animator_server', anonymous=True)
    rate = rospy.Rate(25)
    rospack = rospkg.RosPack()
    path = rospack.get_path('baxter_face_animation') + "/data/upanddown"
    Animation(path)
    while not rospy.is_shutdown():
        rate.sleep()
    

if __name__ == "__main__":
    main()
