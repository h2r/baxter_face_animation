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
from  geometry_msgs.msg import PoseStamped
import tf

class Animation:
    def __init__(self, directory):
        self.fnames = [fname for fname in glob.glob("%s/*" % directory)]
        self.fnames.sort()
        self.images = [cv2.imread(path) for path in self.fnames]

        # print self.fnames
        self.animation_timer = None
        self.current_value = 0
        self.current_idx = 0
        self.set_velocity(1/20.0)
        self.current_target = None

        self._head = baxter_interface.Head()

        self.listener = tf.TransformListener();


        self.image_publisher = rospy.Publisher("/robot/xdisplay", Image,
                                               queue_size=10)

        self.valuex_subscriber = rospy.Subscriber("/eyeGaze/Point/command", PoseStamped, self.set_value)

        # self.valuey_subscriber = rospy.Subscriber("/eyeGaze/valuey/command", PoseStamped, self.set_valuey)

        # self.targety_subscriber = rospy.Subscriber("/eyeGaze/targety/command", PoseStamped, self.set_targety)

        self.valuey_publisher = rospy.Publisher("/eyeGaze/valuey/state", Int32,
                                               queue_size=10)
        self.targety_publisher = rospy.Publisher("/eyeGaze/targety/state", Int32,
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
        if isinstance(value, PoseStamped):
            print "setting value from topic"
            value = value

           # value.header.frame_id - frame that point in reference to
           # transform to screen frame
           # value.pose.point.x value.pose.point.y value.pose.point.z
            #print(value);
        frame = value.header.frame_id
        success = False
        time = rospy.Time.now() + rospy.Duration(2.0)
        trans = []
        rot = []
        while (not success):
            try:
                self.listener.waitForTransform("reference/base", "reference/head_camera", rospy.Time.now(), rospy.Duration(5.0))
                (trans, rot) = self.listener.lookupTransform("/reference/base", "/reference/head_camera", rospy.Time.now())
                success = True;
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                success = False
                print("Transform Error")
            sleep(0.1);

        # print(trans);
        # print(rot);
        # try:
        #    # (trans,rot) = self.listener.lookupTransform(frame, "reference/head_camera", rospy.Time(0))
        #     (trans,rot) = self.listener.lookupTransform("reference/base", "reference/head_camera", rospy.Time(value.header.stamp))
        #     # var mat = 
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print("Transform Error")

        # print self.current_idx
        # self.current_value = value
        # self.current_idx = int(value)#int((value / 100.0) * len(self.images))
        # self.checkrep()
       # return self.publish_image()

    def set_valuex(self, value):
        if isinstance(value, Int32):
            print "setting value from topic"
            value = int(value.data)
        angle = math.radians(value);
        self._head.set_pan(angle, speed=5, timeout=0)

    def set_valuey(self, value):
        if isinstance(value, Int32):
            print "setting value from topic"
            value = int(value.data)
        print self.current_idx
        self.current_value = value
        self.current_idx = int(value)#int((value / 100.0) * len(self.images))
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
        
    # def set_targetx(self, target):
        # if isinstance(target, Int32):
        #     print "setting target from topic"
        #     target = ((target.data)/3.75)+25
        # print "do nothing"
        # self.current_target = target

    def set_targety(self, target):
        if isinstance(target, Int32):
            print "setting target from topic"
            target = target.data;#((target.data)/3.75)+25

        print "setting target", target
        self.current_target = target


    @property
    def image(self):
        return self.images[self.current_idx]

    def publish_state(self):
        # print("donnothing");
        self.valuey_publisher.publish(self.current_value)
        self.targety_publisher.publish(self.current_target)
       
    def timer_cb(self, time):
        self.animate()
        self.publish_state()

    def animate(self):
        if self.current_target != None:
            #print "target", self.current_target, self.current_value
            if self.current_target < self.current_value:
                self.set_valuey(self.current_value - 1)
            elif self.current_target > self.current_value:
                self.set_valuey(self.current_value + 1)
            elif self.current_target == self.current_value:
                self.current_target = None
            else:
                raise ValueError("No target: " + `self.target`)




def main():
    rospy.init_node('animator_server', anonymous=True)
    rate = rospy.Rate(25)
    rospack = rospkg.RosPack()
    path = rospack.get_path('baxter_face_animation') + "/data/DOWN"

    Animation(path)
    while not rospy.is_shutdown():
        rate.sleep()
    

if __name__ == "__main__":
    main()
