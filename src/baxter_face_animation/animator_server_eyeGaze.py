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
#from tf.transformations import euler_from_quaternion
#from tf.transformations import quaternion_matrix

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
        self._head.set_pan(0.0)
        self.listener = tf.TransformListener();

        print("a");
        self.image_publisher = rospy.Publisher("/robot/xdisplay", Image,
                                               queue_size=10)

        self.valuex_subscriber = rospy.Subscriber("/eyeGaze/Point/command2", Pose, self.set_value)

        # self.valuey_subscriber = rospy.Subscriber("/eyeGaze/valuey/command", PoseStamped, self.set_valuey)

        # self.targety_subscriber = rospy.Subscriber("/eyeGaze/targety/command", PoseStamped, self.set_targety)

        self.valuey_publisher = rospy.Publisher("/eyeGaze/valuey/state", Int32,
                                               queue_size=10)
        self.targety_publisher = rospy.Publisher("/eyeGaze/targety/state", Int32,
                                                queue_size=10)


        self.timer = rospy.Timer(rospy.Duration(self.velocity), self.timer_cb)

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

    def quat_to_rot(self, q):
        r1c1 = 1 - 2*(q[2]*q[2] + q[3]*q[3])
        r1c2 = 2*(q[1]*q[2] - q[0]*q[3])
        r1c3 = 2*(q[0]*q[2] + q[1]*q[3])

        r2c1 = 2*(q[1]*q[2] + q[0]*q[3])
        r2c2 = 1 - 2*(q[1]*q[1] + q[3]*q[3])
        r2c3 = 2*(q[2]*q[3] - q[0]*q[1])

        r3c1 = 2*(q[1]*q[3] - q[0]*q[2])
        r3c2 = 2*(q[0]*q[1] + q[2]*q[3])
        r3c3 = 1- 2*(q[1]*q[1] + q[2]*q[2])

        mat = [[r1c1, r1c2, r1c3, 0],   
                   [r2c1, r2c2, r2c3, 0],
                   [r3c1, r3c2, r3c3, 0],
                   [0, 0, 0, 1]]

        #(ax,ay,az) = euler_from_quaternion(q,'xyzs')
        
        #return 
        return mat


    def set_value(self, value):
        print(value)
        if isinstance(value, Pose):
            print "setting value from topic"
            value = value
        # frame = value.header.frame_id
        success = False
        time = rospy.Time.now()
        trans = []
        rot = []
        while (not success):
            try:
                (trans, rot) = self.listener.lookupTransform("/reference/base", "/reference/head_camera", rospy.Time.now())
                success = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                success = False

        mat = self.quat_to_rot(rot);
        mat[0][3] = trans[0];
        mat[1][3] = trans[1];
        mat[2][3] = trans[2];
        

        point = [value.position.x, value.position.y, value.position.z, 1]
	head = [0.15, 0.0, 0.67]
	

        trans_point = np.dot(mat, point);

        x = trans_point[0]
        z = trans_point[2]
        #pheta = math.atan(x/z)
        pheta = math.atan2(value.position.y,value.position.x-0.15)
        print "Pheta " + str(pheta*180/math.pi)
        #while (pheta>90): pheta=2*math.pi
        #while (pheta<-90): pheta=-2*math.pi

        self.set_valuex(pheta);
        self.set_targety(-2.0)
        #self.set_targety(trans_point[1]);

    def set_valuex(self, value):
        if isinstance(value, Int32):
            print "setting value from topic"
            # value = int(value.data)
        #print(value)
        #angle = math.radians(value);
        print "xval = " + str(value*180/math.pi) 
        self._head.set_pan(value, speed=5, timeout=0)

    def set_valuey(self, value):
        self.current_value = value
        self.current_idx = value;
        self.checkrep()
        return self.publish_image()

    def checkrep(self):
        assert 0 <= self.current_idx < len(self.images), self.current_idx
        #assert 0 <= self.current_value < 100, self.current_value
        #assert self.current_target == None or (0 <= self.current_target < 100), self.current_target
    def publish_image(self):
        print(self.current_idx)
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
        target = int((target+2)*(49/4.0))#((target.data)/3.75)+25
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
