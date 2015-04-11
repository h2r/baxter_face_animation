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
	def __init__(self, vid_directory, repeat=0, reverse=False): 

		self.repeat = repeat
		self.reverse = reverse

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
		if self.repetitions > self.repeat: 
			print self.current_frame
			exit()
		if self.reverse: 
			if self.delta == -1 and self.current_frame == 0: 
				self.delta = 1
				self.repetitions += 1
			elif self.delta == 1  and self.current_frame == len(self.images) - 1: 
				self.delta = -1
		else: 
			if (self.current_frame >= len(self.images)):
				self.repetitions += 1
				self.current_frame = 0


				
		image = self.images[self.current_frame]
		msg = cv_bridge.CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
		self.image_publisher.publish(msg)
		self.current_frame += self.delta




def main(): 
	parser = argparse.ArgumentParser(description='Play frames')
	parser.add_argument("frame_dir", help="the directory containing the frames")
	parser.add_argument('--reps', type=int, required=False, help="Number of repetitions")
	parser.add_argument('--reverse', help="reverse animation after playing", action='store_true')
	opts = parser.parse_args()


	rospy.init_node('frame_player', anonymous=True)

	vid_directory = opts.frame_dir
	repeat = opts.reps
	if not repeat: 
		repeat = 0
	fp = FramePlayer(vid_directory, repeat, opts.reverse)
	fp.play()

	reverse_time = 1
	if opts.reverse: 
		reverse_time = 2
	rospy.sleep(reverse_time * repeat * fp.velocity * (len(fp.images)) + fp.velocity * 2)

main()
