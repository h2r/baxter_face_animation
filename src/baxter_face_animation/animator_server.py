#!/usr/bin/env python

import glob
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32
import rospkg

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


        self.image_publisher = rospy.Publisher("/robot/xdisplay", Image,
                                               queue_size=10)
        self.value_subscriber = rospy.Subscriber("/confusion/value/command", Int32, self.set_value)

        self.target_subscriber = rospy.Subscriber("/confusion/target/command", Int32, self.set_target)

        self.value_publisher = rospy.Publisher("/confusion/value/state", Int32,
                                               queue_size=10)
        self.target_publisher = rospy.Publisher("/confusion/target/state", Int32,
                                                queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(self.velocity), self.timer_cb)

 
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
        if isinstance(value, Int32):
            print "setting value from topic"
            value = value.data
        self.current_value = value
        self.current_idx = int((value / 100.0) * (len(self.images)))

        print self.current_idx
        self.checkrep()
        return self.publish_image()

    def checkrep(self):
        assert 0 <= self.current_idx < len(self.images), self.current_idx
        assert 0 <= self.current_value < 100, self.current_value
        assert self.current_target == None or (0 <= self.current_target < 100), self.current_target
    def publish_image(self):
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.image, encoding="bgr8")
        self.image_publisher.publish(msg)
        return self.images[self.current_idx]
        
    def set_target(self, target):
        if isinstance(target, Int32):
            print "setting target from topic"
            target = target.data

        print "setting target", target
        self.current_target = target


    @property
    def image(self):
        return self.images[self.current_idx]

    def publish_state(self):
        self.value_publisher.publish(self.current_value)
        self.target_publisher.publish(self.current_target)
       
    def timer_cb(self, time):
        self.animate()
        self.publish_state()

    def animate(self):
        if self.current_target != None:
            print "target", self.current_target, self.current_value
            if self.current_target < self.current_value:
                self.set_value(self.current_value - 1)
            elif self.current_target > self.current_value:
                self.set_value(self.current_value + 1)
            elif self.current_target == self.current_value:
                self.current_target = None
            else:
                raise ValueError("No target: " + `self.target`)




def main():
    rospy.init_node('animator_server', anonymous=True)
    rate = rospy.Rate(30)
    rospack = rospkg.RosPack()
    path = rospack.get_path('baxter_face_animation') + "/data/avery_v2"

    Animation(path)
    while not rospy.is_shutdown():
        rate.sleep()
    

if __name__ == "__main__":
    main()
