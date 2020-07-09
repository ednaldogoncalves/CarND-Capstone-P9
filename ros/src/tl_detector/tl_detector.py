#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml


STATE_COUNT_THRESHOLD = 1
IMAGE_COUNT = 1  # number of images to skip classifications 
CHECK_DIST_MAX = 100  #  distance to start checking camera image
DESC_LIGHTS = ['Red', 'Yellow', 'Green', 'Unknown']

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
        self.camera_image = None
        self.lights = []

        self.waypoints_2d = None
        self.waypoints_tree = None

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        self.has_image = False
        self.image_count = IMAGE_COUNT  # initialize count of images for classfication
        self.image_state = TrafficLight.UNKNOWN  # initialize 
        self.distance = 100000 #  distance to closest light
        
        '''
        /vehicle/traffic_lights provides the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        
        # Camera images from the car
        # Repository is configured to use the color of the classifier image, but being used raw images.
        # In raw images there is no data loss when converting them to a new color scheme		
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
 
        # Launch file will distinguish either site or sim
        config_string = rospy.get_param("/traffic_light_config")

        # self.config is a  dictionary
        self.config = yaml.load(config_string)  
 
        self.bridge = CvBridge()

        # The classifier does not need to visually see any schema
        # of specific colors as a person would
        # It is useful to have as much data as possible in the Classifier        
        self.light_classifier = TLClassifier()  # site or simulator
        self.listener = tf.TransformListener()

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg


    def waypoints_cb(self, lane):
        self.base_waypoints = lane
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in lane.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        self.lights = msg.lights
        if not self.has_image:
            self.publish_upcoming_red_light()


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        self.publish_upcoming_red_light()
        

    def publish_upcoming_red_light(self):
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        # Light status based on traffic light processing 
        light_wp, state = self.process_traffic_lights()
  
        # We're trying to make sure that this light
        # is staying pretty consistent before we're taking any action,
        # because the state of the light maybe is changed from green to yellow, or yellow to red.
        if self.state != state:
            self.state_count = 0
            self.state = state
        # That looks like it's happening when the state count is greater then
        # some threshold here and then we're saving the last state from what the state was			
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            # We publish the location of the RED traffic light,
            # because we're really only interested on this,
            # then everything else we can continue driving through            
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            rospy.loginfo("Publishing new waypoint")
        else:
            # Publishing the last waypoint and then state count plus on for our counter
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            rospy.loginfo("Publishing old waypoint")
        self.state_count += 1


    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            x, y: position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoints_tree.query([x,y],1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # For testing in simulator, just return the light state
        if not self.has_image:
            rospy.loginfo("Did not have image")
            return light.state
      
        # check image only within distance of CHECK_DIST
        if (self.distance <= CHECK_DIST_MAX ):  
            if (self.image_count < IMAGE_COUNT):              
                self.image_count += 1
                return self.image_state
   
            #Get classification           
            self.distance = len(self.base_waypoints.waypoints) # reset distance 

            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")       
            self.image_state  = self.light_classifier.get_classification(cv_image)    

            if (self.image_state == TrafficLight.RED): # if red light , skip a couple of check
                self.image_count = 0
            
            #rospy.logwarn("return self.image_state={}".format(self.image_state))
            rospy.logwarn("Traffic light state: %r" % (DESC_LIGHTS[self.image_state]))
            return self.image_state
        else:
            self.image_count = IMAGE_COUNT
            rospy.logwarn('No traffic light')
            return TrafficLight.UNKNOWN # light too far away
       

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # It gives us the nearest traffic light, but each traffic light comes with a traffic line,
        # but what is the stop line for that traffic light.
        # We want to get the index of the traffic line closest to us

        closest_light = None
        stop_wp_idx = -1 # waypoint index for upcoming stop line

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            
            #TODO find the closest visible traffic light (if one exists)

            # Iterate through the list of traffic lights to find the closest one,
            # then we start off with the difference as big as it possibly could be			
            diff = len(self.base_waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                stop_pos = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(stop_pos[0], stop_pos[1])

                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if 0 <= d < diff:
                    diff = d
                    closest_light = light
                    stop_wp_idx = temp_wp_idx

        # We can just see what that state of the light is
        # after we've grabbed the closest_light       
        if closest_light:
            self.distance = diff
            state = self.get_light_state(closest_light)
            
            rospy.logwarn("Distance to the next traffic light: %r " % (diff))
            return stop_wp_idx, state

        # If you don't have a traffic light and you want to be returning negative             
        # and if you detect a traffic light and you cant't tell what its st                
        # then you want to be returning a traffic light
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
