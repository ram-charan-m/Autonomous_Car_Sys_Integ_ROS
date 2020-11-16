#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
from twist_controller import Controller

'''
NODE DESCRIPTION:
Built after partial implementation of `waypoint_updater` node.
Node subscribes to `/twist_cmd` message which provides the proposed linear and angular velocities.
Also subscribes to '/dbw_enabled', DBW mode or manual, and '/current_velocity'.
Reset values whil not /dbw_enabled prevents PID integral windup.
Two launch files for this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.
The proposed throttle, brake, and steer values are published on the various publishers created in the `__init__` function.
'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # control input publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # creating 'Controller` object
        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband, decel_limit, 
                                     accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle)        

        # Subscribing to all the topics needed
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb) #drive-by-wire or manual
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb) # ctrl inputs
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb) # current velocity        
        
        self.current_vel = None
        self.cur_ang_vel = None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.throttle = self.steering = self.brake = 0 

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz - based on autoware freq of operation
        while not rospy.is_shutdown():
            # getting predicted throttle, brake, and steering using `twist_controller`
            if None not in (self.current_vel, self.linear_vel, self.angular_vel): 
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                                   self.dbw_enabled,
                                                                                   self.linear_vel,
                                                                                   self.angular_vel)
            # publising in dbw mode only
            if self.dbw_enabled:
              self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()            
    
    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg
        
    def twist_cb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z
        
    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x
            
    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

if __name__ == '__main__':
    DBWNode()