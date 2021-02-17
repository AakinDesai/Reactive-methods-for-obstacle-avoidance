#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:

    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) 
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10) 
    
    def preprocess_lidar(self, ranges, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
	i = 0
        proc_ranges = list(ranges)

	iv1 = int((math.radians(90) - data.angle_min)/(data.angle_increment))  
	iv2 = int((math.radians(-90) - data.angle_min)/(data.angle_increment)) 

	proc_ranges[0: iv2]= [0]*iv2  
	proc_ranges[iv1: len(proc_ranges)]= [0]*(len(proc_ranges)-iv1) 

	proc_ranges[proc_ranges > data.range_max] = data.range_max
	proc_ranges[proc_ranges < data.range_min] = 0

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
	i = 0
        gs = -1 
	ge = -1 
	bg = 0 
	bs = 0 
	be = 0 

	while i <= len(free_space_ranges)-1: 
	    if (free_space_ranges[i] > 2) and (gs == -1): 
		gs = i
	    elif (free_space_ranges[i] <= 2) and (gs != -1):
		ge = i
		if (ge - gs) > bg:
		    bg = ge - gs
	    	    bs = gs
	    	    be = ge
		gs = -1
	    i = i + 1
	arr = [bs, be]
        return arr
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        best = (start_i + end_i)/2 
        return best


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = list(data.ranges)

        # the closest value to the lidar
	mv = min(ranges)	

        #Eliminate all points inside 'bubble' (set them to zero) 
	mvi = ranges.index(mv)	
        angle = 0.1/mv
	bub = int(angle/data.angle_increment)	
	ranges[mvi + bub: mvi-bub]=[0]*(-bub*2)
        ranges = self.preprocess_lidar(ranges, data)


        #Find max length gap 
	arr = self.find_max_gap(ranges)
	bs=arr[0]  
	be=arr[1]
	bg = be - bs 
		
        #Find the best point in the gap 
	bp = self.find_best_point(bs, be, ranges) 
	velocity=4

        #Publish Drive message
	drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
	iv = bp
	ra =  ((data.angle_increment)*(iv) + data.angle_min) 
        drive_msg.drive.steering_angle = ra		
        drive_msg.drive.speed = velocity	
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
