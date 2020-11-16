#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math


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

LOOKAHEAD_WPS = 40 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5
PUBLISH_RATE = 30
STOPPING_LEEWAY = 3


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) # subscriber to store pose, gets updated freq
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # subscriber to get base wasy points
        # setup as latched subscriber, meaning once the callback is called it doesnt send base_waypoints anymore
        # As publishers operate in a frequency, it will be very inefficient to copy constant base waypoints

        #TODO: Adding a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
#         rospy.Subscriber('/obstacle_waypoint', <msg_type>, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # Initializing other member variables used below
        self.pose = None
        self.base_lane = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        #rospy.spin()
        self.loop() # adds cycling action at specified frequency
        
    def loop(self):
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree: # if we have pose, base wp
                closest_waypoint_idx = self.get_closest_waypoint_idx() # getting closest wp
                self.publish_waypoints(closest_waypoint_idx) #publishing message
            rate.sleep()
    
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1] # returns closest distance and index 
        # index is same as order in which its added to tree i.e., index in base wps

        # checking if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        # imagining a hyperplane perpendicular to the vetor joining cl and prev
        # calculating dot prod of vehi pose wrt hyperplane
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0: # when waypoints is behind us
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d) # updating closest point to next point
            #rospy.logwarn("closest_idx={}".format(closest_idx))
        return closest_idx
    
    def publish_waypoints(self, closest_idx): # publish function
        final_lane = self.generate_lane() # message type needs to be a lane
        #lane.header = self.base_waypoints.header
        self.final_waypoints_pub.publish(final_lane)
        
    def generate_lane(self): # generates lane type data based on traffic light detection
        lane = Lane()
        
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx): # no index or outside our concern limit
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp =[]
        for i, wp in enumerate(waypoints):
            p = Waypoint() # new waypoint message
            p.pose = wp.pose # setting new pose to base wp pose
            
            stop_idx = max(self.stopline_wp_idx - closest_idx - STOPPING_LEEWAY, 0) # 2 wp back from traffic line wp to avoid crossing line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.0:
                vel = 0
            
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x) # avoid large vel at large dist
            temp.append(p)
            
        return temp
    
    def pose_cb(self, msg):
        self.pose = msg

    # this is call back function thats called everytime the subscriber publishes
    # it must store the base waypoints in a container(KDTree)
    def waypoints_cb(self, waypoints): 
        self.base_lane = waypoints # storing base in waypoints obj
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)                 

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message
        self.stopline_wp_idx = msg.data
        

    def obstacle_cb(self, msg):
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
