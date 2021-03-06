#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import time
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
	
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
	self.light_classifier = None # debug dbw_node.py
	self.debug_mode = True
   	self.waypoints_2d = None
	self.waypoint_tree = None
	
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
    	rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) #sub1
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # sub2
	rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb) #sub3 
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1) #sub6

        config_string = rospy.get_param("/traffic_light_config")
	
	#rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_enabled_cb)
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoints', Int32, queue_size=1)

        self.bridge = CvBridge()
        # using debug flag to turn on and off self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
	if not self.debug_mode:
	    self.light_classifier = TLClassifier()
	    rospy.logwarn("Classifier being initialized")
	else:
	    self.light_classifier = None
	    rospy.logwarn("Debug mode dbw_node. no classification")
	    
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
	
	# collecting training images, settings for TL_Classifier
	self.collect_images = False
	self.ros_bag_mode = False
	self.image_max_dist = 60
	self.image_min_dist = 5
	self.image_interval = 2
	self.image_last_light = None
	self.image_last_dist = 0
	self.dbw_enabled = False
	self.class_time = rospy.get_time()
	self.waypoints_2d = None
        #rospy.spin()
        self.loop()
    def loop(self):
        # publishing frequency of 35 hertz
		rate =rospy.Rate(3) # waypoint follower is running at 30hz 
		while not rospy.is_shutdown():
            #if self.pose and self.base_waypoints:
			light_wp, state = self.process_traffic_lights()
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
			rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d: # and self.dbw_enabled:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
		
    def traffic_cb(self, msg):
        self.lights = msg.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
	#self.class_interval = rospy.get_time() - self.class_time  
	#if self.class_interval < 1.5 and self.debug_mode:
        #    return #classifying interval too short
	
        #light_wp, state = self.process_traffic_lights()
	#self.class_time = rospy.get_time()
	
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
	'''	
	# previous state to detect state change
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
	'''
    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        #TODO implement  ---- Direction and advisement from walkthrough video 
	#x = self.pose.pose.position.x
	#y = self.pose.pose.position.y
	if self.waypoint_tree:
	    closest_idx = self.waypoint_tree.query([x,y],1)[1]
            #rospy.logwarn("closest_idx is {0}".format(closest_idx))
  	    return closest_idx
	else:
	    return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #if(not self.has_image):
        #    self.prev_light_loc = None
        #    return False

        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification  #testing
        return light.state #self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	closest_light = None        
	line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection self.get_closest_waypoint
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x,self.pose.pose.position.y)
	    #rospy.logwarn("car_position is: {0}".format(car_position))	

	    #rospy.logwarn("pose_x: {0}".format(self.pose.pose.position.x))
	    #rospy.logwarn("pose_y: {0}".format(self.pose.pose.position.y))
        #TODO find the closest visible traffic light (if one exists)
		# length of waypoints
	    wp_len = len(self.waypoints.waypoints)
	    # iterate through waypoints to find closest one
	    for i, light in enumerate(self.lights):
	        # find line of pts to stop before traffic light
     	        line = stop_line_positions[i] # positions of stop lines
		#rospy.logwarn("line 1 is {0}".format(line[0]))
		#rospy.logwarn("line 2 is {0}".format(line[1]))
	        comparison_idx = self.get_closest_waypoint(line[0], line[1])
			# number of indices between car and closest to stop line	        
		#rospy.logwarn("comparison_idx is".format(comparison_idx))
		#rospy.logwarn("car_position is".format(car_position))		
		dist = comparison_idx - car_position 			
	    		# smallest difference between position and stop line position
	        if dist>=0 and dist < wp_len:
	            wp_len = dist
		    closest_light = light
	            car_position = comparison_idx

        if closest_light:
	    state = self.get_light_state(closest_light)
	    return car_position, state	
	#if light:
        #    state = self.get_light_state(light)
        #    return light_wp, state
        else:
	#self.waypoints = None
	# if there isn't a close by traffic light, -1 to ensure normal driving
            return -1, TrafficLight.UNKNOWN
	
    def pose_cb(self, msg):
        # TODO: Implement
	    self.pose = msg # 
    def dbw_enabled_cb(self,msg):
	    pass #self.dbw_enabled = msg

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
