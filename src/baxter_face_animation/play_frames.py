#!/usr/bin/python
import glob
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32
import rospkg
import argparse

from os import listdir
from os.path import isfile, join
from os import sys
from PyQt4.QtCore import SIGNAL, QTimer

class FramePlayer: 
	def __init__(self, vid_directory, repeat=0, reverse=False, pause=0.0): 

		self.repeat = repeat
		self.reverse = reverse
		self.pause = pause

		self.image_publisher = rospy.Publisher("/robot/xdisplay", Image, queue_size=100)
		files = sorted([ f for f in listdir(vid_directory) ])
		self.images = [cv2.imread(vid_directory + f) for f in files]

		self.repetitions = 0
		self.velocity = (1/20.0)
		self.current_frame = 0
		self.delta = 1


	def play(self): 
		self.timer = rospy.Timer(rospy.Duration(self.velocity), self.play_cb)

	def play_cb(self, time): 
		if self.reverse: 
			if self.delta == -1 and self.current_frame == -1: 
				self.delta = 1
				self.repetitions += 1
				self.current_frame = 0
				if self.pause: rospy.sleep(self.pause)
			elif self.delta == 1  and self.current_frame == len(self.images): 
				self.current_frame -= 1
				self.delta = -1
				if self.pause: rospy.sleep(self.pause)
		else: 
			if (self.current_frame >= len(self.images)):
				self.repetitions += 1
				self.current_frame = 0
				if self.pause: rospy.sleep(self.pause)

		if self.repetitions > self.repeat: 
			exit()

				
		image = self.images[self.current_frame]
		msg = cv_bridge.CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
		self.image_publisher.publish(msg)
		self.current_frame += self.delta

	def duration(self): 
		repeat = self.repeat
		if not self.repeat: 
			repeat = 0
		pause = self.pause
		if not self.pause: 
			pause = 0.0
		reverse_time = 1
		if self.reverse: 
			reverse_time = 2
		return 1+ (reverse_time * repeat *  (pause+ 1 +  self.velocity * (len(self.images))))





def main(): 
	parser = argparse.ArgumentParser(description='Play frames')
	parser.add_argument("frame_dir", help="the directory containing the frames, typically starts with 'data'")
	parser.add_argument('--reps', type=int, required=False, help="Number of repetitions")
	parser.add_argument('--reverse', help="reverse animation after playing", action='store_true')
	parser.add_argument('--pause', help="time to pause after completion", type=float)
	opts = parser.parse_args()


	rospy.init_node('frame_player', anonymous=True)

	rospack = rospkg.RosPack()
	path = rospack.get_path('baxter_face_animation')

	vid_directory = path + '/' + opts.frame_dir
	repeat = opts.reps
	pause = opts.pause
	fp = FramePlayer(vid_directory, repeat, opts.reverse, pause)
	fp.play()

	rospy.sleep(fp.duration())

main()
