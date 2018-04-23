#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math
import numpy as np
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
TARGET_SPEED = 20 # Unit MPH
ONE_MPH = 0.44704 # 1MPH = 0.44707m/s
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
		rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
		self.pose = None
		self.waypoints_2d = None
		self.base_waypoints = None
		self.waypoint_tree = None
		
		self.loop()
        rospy.spin()
    
	def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
		if not self.waypoints_2d:
			self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
			self.waypoint_tree = KDTree(self.waypoints_2d)
    
	def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_wp = msg

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_wp = msg

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
	
	def get_closest_waypoint_id(self):
		x = self.pose.pose.position.x
		y = self.pose.pose.position.y
		closest_wp_id = self.waypoint_tree.query([x,y],1)[1]
		
		closest_wp_coord = np.array(self.waypoints_2d[closest_wp_id])
		prev_wp_coord = np.array(self.waypoints_2d[closest_wp_id-1])
		pos_coord = np.array([x,y])
		
		val = np.dot(closest_wp_coord - prev_wp_coord,pos_coord - closest_wp_coord)
		if val > 0:
			closest_wp_id = (closest_wp_id + 1) % (len(self.waypoints_2d))
		return closest_wp_id

	def publish_final_waypoints(self, closest_idx):
		lane = Lane()
		lane.header = self.base_waypoints.header
		lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]
        #In the planning path red light is detected.
        if self.traffic_wp >= 0:
            offset = self.traffic_wp - closest_idx
            for i, waypoint in enumerate(lane.waypoints):
                dist = self.distance(lane.waypoints, i, offset)
                if i >= offset:
                    pred_speed = 0.0
                elif distance < 15:
                    pred_speed = math.sqrt(2 * MAX_DECEL * dist)
                    if pred_speed <= 1:
                        pred_speed = 0.0
                    pred_speed = min(pred_speed, self.get_waypoint_velocity(waypoint))

                lane.waypoints[i].twist.twist.linear.x = pred_speed

		self.final_waypoints_pub.publish(lane)
	
	def loop(self):
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			if self.pose and self.base_waypoints:
				closest_idx = self.get_closest_waypoint_id()
				self.publish_final_waypoints(closest_idx)
			rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
