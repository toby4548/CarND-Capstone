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
import math

STATE_COUNT_THRESHOLD = 3
USE_MODEL = False
PRINT_PREDICTION_LOGGING = False

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_num = None
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

        if USE_MODEL:
            self.light_classifier = TLClassifier()
        
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.waypoints_num = len(msg.waypoints)

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

    def path_distance(self, p1, p2):
        """
        return the distance between two position msg
        """
        dl = lambda a,b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        return dl(p1,p2)

    def distance(self, p1, p2):
        return math.sqrt((p2.position.x-p1.position.x)**2 + (p2.position.y-p1.position.y)**2)
     
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = -1
        closest_dist = 10e10
        
        if self.waypoints is None:
            return closest_idx
        else:
            for i in range(self.waypoints_num):
                dist = self.distance(self.waypoints[i].pose.pose, pose)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_idx = i

            return closest_idx

    def get_light_state(self):
        """Determines the current color of the traffic light


        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        if USE_MODEL:
            return self.light_classifier.get_classification(cv_image)
        else: 
            return TrafficLight.UNKNOWN

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position_idx = self.get_closest_waypoint(self.pose.pose)

            
            # Get closest stop line
            closest_idx_diff = 10e10
            closest_stop_line_pose = None
            closest_stop_line_idx = None
            for i in range(len(stop_line_positions)):
                stop_flag = False
                state = TrafficLight.UNKNOWN
                stop_line_pose = PoseStamped()
                stop_line_pose.pose.position.x = stop_line_positions[i][0]
                stop_line_pose.pose.position.y = stop_line_positions[i][1]
                stop_line_idx = self.get_closest_waypoint(stop_line_pose.pose)
                idx_diff = stop_line_idx - car_position_idx
                if idx_diff < 0:
                    idx_diff += self.waypoints_num

                if idx_diff < closest_idx_diff:
                    closest_idx_diff = idx_diff
                    closest_stop_line_pose = stop_line_pose
                    closest_stop_line_idx = stop_line_idx

                if not USE_MODEL:
                    stop_line_state = self.lights[i].state
                     
                    if stop_line_state != TrafficLight.RED:
                        stop_flag = False
                        state = TrafficLight.UNKNOWN
                    else:
                        stop_flag = True
                        state = stop_line_state
                
                else: 
                    #Change this part if use model for prodiction
                    #TODO find the closest visible traffic light (if one exists)
                    pdistance =  self.path_distance(self.pose.pose.position,
                                         closest_stop_line_pose.pose.position)
                    

                    if  pdistance <100.0:                      
                        stop_line_state = self.get_light_state()
			if PRINT_PREDICTION_LOGGING:                        
				rospy.logwarn(str(stop_line_state))     
                        if stop_line_state != TrafficLight.RED:
                            stop_flag = False
                            state = TrafficLight.UNKNOWN
                        else:
                            stop_flag = True
                            state = stop_line_state
                    else:
                        stop_flag = False
                        state = TrafficLight.UNKNOWN

            if stop_flag == True:
                return closest_stop_line_idx, state
            else:
                return -1, TrafficLight.UNKNOWN
            
        else:            
            return -1, TrafficLight.UNKNOWN 

        return -1, TrafficLight.UNKNOWN 


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
