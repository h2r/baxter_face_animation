#!/usr/bin/env python

import glob
import copy
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32, String
import rospkg


class Animation:
    def __init__(self, directory):
        self.animation_timer = None
        self.current_value = 0
        self.current_idx = 0
        self.set_velocity(1/50.0)
        #self.current_target = None
        self.current_emotion = "nonchalant"
        self.old_emotion = "nonchalant"
        #attentive, confused
        self.direction = True
        self.emotion_list = ["nonchalant","heard_an_order","upanddown"]
        self.once = True


        #set current directory to nonchalant, read images from the directory and store in map
        self.subdirectory = "nonchalant/"
        self.complete_path = directory + self.subdirectory
        self.fnames = [fname for fname in glob.glob("%s/*" % self.complete_path)]
        self.fnames.sort()
        self.images = [cv2.imread(path) for path in self.fnames]
        self.image_map = dict()
        self.image_map["nonchalant"] = list(self.images)



        # read images from heard_and_understand the directory and store in map
        temp_subdirectory = "heard_an_order/"
        temp_complete_path = directory + temp_subdirectory
        temp_fnames = [fname for fname in glob.glob("%s/*" % temp_complete_path)]
        temp_fnames.sort()
        temp_images = [cv2.imread(path) for path in temp_fnames]
        self.image_map["heard_an_order"] = list(temp_images)



        # read images from heard_and_understand the directory and store in map
        temp_subdirectory = "upanddown/"
        temp_complete_path = directory + temp_subdirectory
        temp_fnames = [fname for fname in glob.glob("%s/*" % temp_complete_path)]
        temp_fnames.sort()
        temp_images = [cv2.imread(path) for path in temp_fnames]
        self.image_map["upanddown"] = list(temp_images) 







        self.image_publisher = rospy.Publisher("/robot/xdisplay", Image,
                                               queue_size=10)
        self.value_subscriber = rospy.Subscriber("/confusion/value/command", Int32, self.set_value)

        #self.target_subscriber = rospy.Subscriber("/confusion/target/command", Int32, self.set_target)

        self.emotion_subscriber = rospy.Subscriber("/emotion", String, self.set_emotion)

        self.value_publisher = rospy.Publisher("/confusion/value/state", Int32,
                                               queue_size=10)
        self.emotion_publisher = rospy.Publisher("/confusion/emotion/state", String,
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

        # print self.current_idx
        self.checkrep()
        return self.publish_image()

    def checkrep(self):
        assert 0 <= self.current_idx < len(self.images), self.current_idx
        assert 0 <= self.current_value < 100, self.current_value
        assert 0 <= self.current_value > -1, self.current_value
        #assert self.current_target == None or (0 <= self.current_target < 100), self.current_target
    def publish_image(self):
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.image, encoding="bgr8")
        self.image_publisher.publish(msg)
        return self.images[self.current_idx]
        
    # def set_target(self, target):
    #     if isinstance(target, Int32):
    #         print "setting target from topic"
    #         target = target.data

    #     print "setting target", target
    #     self.current_target = target

    def set_emotion(self, emotion_string):
        temp_flag = False
        if isinstance(emotion_string, String):
            emotion_string = emotion_string.data
            temp_flag = True
            # print "first check point"
            # print "setting emotion from topic"
        


        if emotion_string != self.current_emotion:
            temp_index = None
            try:
                temp_index = self.emotion_list.index(emotion_string)
                if temp_flag:
                    print "setting emotion from topic"
            except ValueError:
                if temp_flag:
                    print "set emotion not found"
            if temp_index is not None:
                print "setting emotion", emotion_string
                self.current_emotion = emotion_string
                try:
                    self.images = self.image_map[emotion_string]
                except KeyError:
                    # this should have confused ideally
                    self.images = self.image_map["nonchalant"]
                self.set_value(0)
            # self.subdirectory = temp_emotion
            # self.complete_path = directory + self.subdirectory
            # self.fnames = [fname for fname in glob.glob("%s/*" % self.complete_path)]
            # self.fnames.sort()
            # self.images = [cv2.imread(path) for path in self.fnames]
            

        


    @property
    def image(self):
        return self.images[self.current_idx]

    def publish_state(self):
        self.value_publisher.publish(self.current_value)
        self.emotion_publisher.publish(self.current_emotion)
        # self.target_publisher.publish(self.current_target)
       
    def timer_cb(self, time):
        self.animate()
        self.publish_state()

    def animate(self):
        if(self.current_emotion == "nonchalant"):
            if self.current_value == 99:
                self.direction = False
            elif self.current_value == 0:
                self.direction = True
            if self.direction:
                self.set_value(self.current_value + 1)
            else:
                self.set_value(self.current_value - 1)
        else:
            if(self.once):
                if self.current_value == 99:
                    self.direction = False
                    self.once = False
                elif self.current_value == 0:
                    self.direction = True
                if self.direction:
                    self.set_value(self.current_value + 1)
                else:
                    self.set_value(self.current_value - 1)
            else:
                self.set_emotion("nonchalant")
                self.once = True



        # #
        # if self.current_target != None:
        #     print "target", self.current_target, self.current_value
        #     if self.current_target < self.current_value:
        #         self.set_value(self.current_value - 1)
        #     elif self.current_target > self.current_value:
        #         self.set_value(self.current_value + 1)
        #     elif self.current_target == self.current_value:
        #         self.current_target = None
        #     else:
        #         raise ValueError("No target: " + `self.target`)




def main():
    rospy.init_node('animator_server', anonymous=True)
    rate = rospy.Rate(30)
    rospack = rospkg.RosPack()
    path = rospack.get_path('baxter_face_animation') + "/data/"

    Animation(path)
    while not rospy.is_shutdown():
        rate.sleep()
    

if __name__ == "__main__":
    main()
