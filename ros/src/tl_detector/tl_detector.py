#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 200

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

	def pose_cb(self, msg):
        self.pose = msg
		
	def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

	def traffic_cb(self, msg):
		self.lights = msg.lights

	def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
		pos_x = pose.position.x	#current car position
		pos_y = pose.position.y	#current car position
		
		num_wp = len(self.waypoints.waypoints)
		
		dist = [pos_x*self.waypoints.waypoints[i].pose.pose.position.x + pos_y*self.waypoints.waypoints[i].pose.pose.position.y for i in range(num_wp)]
        closest_wp_id = dist.index(min(dist))

		clst_wp_x = self.waypoints.waypoints[closest_wp_id].pose.pose.position.x
		clst_wp_y = self.waypoints.waypoints[closest_wp_id].pose.pose.position.y
		prev_clst_wp_x = self.waypoints.waypoints[closest_wp_id-1].pose.pose.position.x
		prev_clst_wp_y = self.waypoints.waypoints[closest_wp_id-1].pose.pose.position.y
		
		if (clst_wp_x-prev_clst_wp_x)*(pos_x - clst_wp_x) + (clst_wp_y-prev_clst_wp_y)*(pos_y-clst_wp_y)>0:
			closest_wp_id = (closest_wp_id + 1) % len(self.waypoints.waypoints)
		return closest_wp_id

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
		"""
		return light.state
		
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose) #Closest waypoint index to current car position.

        #TODO find the closest visible traffic light (if one exists)
		
		#Generate a list of closest waypoints ahead ID for traffice light
		clst_wp_toTL = [self.get_closest_waypoint(self.lights[i].pose.pose) for i in range(len(self.lights))]
		#Generate a list of distance from car wp id to each light wp id
		OFFSET_list = [1000000 if wp_t1-car_position < 0 else wp_tl-car_position for wp_t1 in clst_wp_toTL]
		#Find which traffic light is now visible and closest to car.
		light_id = OFFSET_list.index(min(OFFSET_list))		
		ID_clstWP_toTL = clst_wp_toTL[light_id]
		#Find closest traffic light to car.
		light = self.lights[light_id]
		
		if light:
			state = self.get_light_state(light)
			stop_line_pos = Pose()
			stop_line_pos.position.x = stop_line_positions[light_id][0]
			stop_line_pos.position.y = stop_line_positions[light_id][1]
			if (car_position<=ID_clstWP_toTL<car_position+LOOKAHEAD_WPS)
				light_wp = self.get_closest_waypoint(stop_line_pos)-1
			return light_wp, state
		return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
