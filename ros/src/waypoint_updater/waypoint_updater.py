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

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish.
                    # Those are the first 200 waypoints in front of the car.

RATE = 50           # Orig: 50Hz, publishing frequency
STOP_AHEAD = 3      # Buffer waypoints that one stop line is ahead 
MAX_DECEL = 2       # Maximum deceleration 
SMOOTH_DECEL = 1. / LOOKAHEAD_WPS # Smooth deceleration

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None 

        self.stopline_wp_idx = -1

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Using a subscriber and the base waypoints
        # If you look at the rospy subscriber,
        # it has base waypoints, that's the topic.
        # Lane => is the message type that's coming over the topic.
        # waypoints_cb => is the waypoints callback,
        # that's the function that basically gets called every time
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Defined a loop function that gives us
        # basically the reason we're doing this is because this
        # gives us control over the publishing frequency
        self.loop()

    def loop(self):

        # We want to target 50 hertz, 10 stop
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get closest waypoint
                final_lane = self.generate_lane()               
                self.final_waypoints_pub.publish(final_lane)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        if self.waypoints_tree:
            # To get the coordinates of car
            # The first one there does we only want to return one item
            # The closest point in KDTree to our query item
            # The second "1" that the query will return the position and also the index
            # and the index is in the same order as into the KDTree
            # Put in all those 2D coordinates for waypoints and what we're trying to get
            # if basically the index of that coordinate
            closest_idx = self.waypoints_tree.query([x, y], 1)[1]

            # Check if closest is ahead or behind vehicle
            # To get closest waypoint from waypoint tree query,
            # but wants to make sure that point is in front of the car
            closest_coord = self.waypoints_2d[closest_idx]
            prev_coord = self.waypoints_2d[closest_idx-1]

            # Equation for hyperplane through closest_coords
            # If it is behind the car, if we've determined that the closest
            # waypoint is behind us, we just take the next one
            closest_vect = np.array(closest_coord)
            prev_vect = np.array(prev_coord)
            pos_vect = np.array([x, y])

            val = np.dot(closest_vect-prev_vect, pos_vect-closest_vect)

            # We take the next one, but we modulo the length of the total waypoint list
            # Meaning the way point is actually behind us
            # then we take the closest index plus one, but modulo the length of waypoint 2D
            if val > 0:
                closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
            return closest_idx
        else:
            rospy.logwarn("get_closest...return -1")
        return -1

    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_waypoints.header

        # We have our closest index and the index of the closest waypoint ahead of the car
        # and the message type needs to be a new lane object or a new lane message
        # and the lane header is the same as the base waypoints header
        # The waypoints for that lane should be the waypoints from your base waypoints,
        # but sliced so that it goes from your closest index to the closest index plus however
        # many would look ahead points you want

        closest_idx = self.get_closest_waypoint_idx()
        if (closest_idx == -1):
            rospy.logwarn("closest_idx == -1 lane return None")
            return None

        farthest_idx = closest_idx + LOOKAHEAD_WPS
        final_waypoints = self.base_waypoints.waypoints[closest_idx: farthest_idx]

        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints =  final_waypoints
        else:
            #rospy.logwarn("decelerate_waypoints...")
            lane.waypoints = self.decelerate_waypoints(final_waypoints, closest_idx)
              
        return lane


    def decelerate_waypoints(self, waypoints, closest_idx):
        # Adjustment of target speeds of waypoints that lead to red traffic lights
        # or other obstacles, in order to bring the vehicle to a smooth and complete stop.
        # Smooth decrease in speed before the stop point.
        temp_waypoints = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - STOP_AHEAD, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist) + (i * SMOOTH_DECEL) # + linear deceleration
            if vel < 1.0:
                vel = 0.0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp_waypoints.append(p)

        return temp_waypoints

    def pose_cb(self, msg):
        # TODO: Implement
        # Just stores the car's pose
        self.pose = msg

    # Callback function
    def waypoints_cb(self, lane):
        # TODO: Implement

        # Basically just stores thes in the object
        # The basic idea is that we want to take a chunk of
        # these waypoints and use the first 200 that are in front of the car as reference
        self.base_waypoints = lane
        
        if not self.waypoints_2d:

            # We've converted the waypoints to just the 2D coodinates for each waypoint
            # Will give us a list of a bunch of 2D coordinates for each waypoint
            # We use that to construct a KDTree
            # We've done that with a single callback and now waht other information,
            # are we getting from subscribers, an important one is probably the pose of the car
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in lane.waypoints]

            # KDTree is a data structure that'll allow you to look up
            # the closest point in space really efficiently
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        # Gets the linear velocity (x-direction) for a single waypoint
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        # Sets the linear velocity (x-direction) for a single waypoint 
        # in a list of waypoints. Here, waypoints is a list of waypoints, 
        # waypoint is a waypoint index in the list, and velocity is the desired velocity.
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        # The distance between two waypoints in a list along the piecewise linear arc 
        # connecting all waypoints between the two. Here, waypoints is a list of waypoints, 
        # and wp1 and wp2 are the indices of two waypoints in the list. 
        # To determine the velocities for a sequence of waypoints leading up to a red light
        # (the velocities should gradually decrease to zero starting some distance from the light).
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
