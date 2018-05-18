#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import lowpass
import math
import numpy as np
import tf
import copy

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node',log_level=rospy.DEBUG)
        rospy.logdebug("DBW Node Init Started")
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

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.pose=None
        self.waypoints=None
        self.twist_cmd=None
        self.current_vel=None
        self.angular_vel=None
        self.linear_vel=None
        self.dbw_enabled=False
        self.throttle=0
        self.steering=0
        self.brake=0
        self.frame_id=None

        # TODO: Create `Controller` object
        self.controller = Controller(vehicle_mass=vehicle_mass,
                                     fuel_capacity=fuel_capacity,
                                     brake_deadband=brake_deadband,
                                     decel_limit=decel_limit,
                                     accel_limit=accel_limit,
                                     wheel_radius=wheel_radius,
                                     wheel_base=wheel_base,
                                     steer_ratio=steer_ratio,
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle)


        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity',TwistStamped,self.velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)
        self.tf_listener = tf.TransformListener()

        rospy.logdebug("DBW Init Complete")
        self.loop()

    def get_cte(self):
        # no need to test time_last_cmd since it is assigned together with twist_cmd
        cte=0
        if self.waypoints != None and self.twist_cmd != None and self.current_vel != None:

            # Create lists of x and y values of the next waypoints to fit a polynomial
            x = []
            y = []
            i = 0
            # Due to race conditions, we need to store the waypoints temporary
            temp_waypoints = copy.deepcopy(self.waypoints)
            #rospy.logdebug("FRAME ID is %s", self.frame_id)
            while len(x) < 20 and i < len(temp_waypoints.waypoints):
                # Transform waypoint to car coordinates
                temp_waypoints.waypoints[i].pose.header.frame_id = self.frame_id
                self.tf_listener.waitForTransform("/base_link", "/world", rospy.Time(0), rospy.Duration(10))
                transformed_waypoint = self.tf_listener.transformPose("/base_link", temp_waypoints.waypoints[i].pose)
                # Just add the x coordinate if the car did not pass the waypoint yet
                if transformed_waypoint.pose.position.x >= 0.0:
                    x.append(transformed_waypoint.pose.position.x)
                    y.append(transformed_waypoint.pose.position.y)
                i += 1
            if(len(x)==0 or len(y)==0):
                rospy.logdebug("x or y is malformed")
                return 0
            coefficients = np.polyfit(x, y, 3)
            # We have to calculate the cte for a position ahead, due to delay
            cte = np.polyval(coefficients, 0.7 * self.current_vel)
            cte *= abs(cte)
            rospy.logdebug('cte: %s', cte)
        return cte


    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            #str="Linear Vel={} Angular Vel={} current vel={}".format(self.linear_vel,self.angular_vel,self.current_vel)
            #rospy.logdebug(str)
            if not None in (self.current_vel,self.linear_vel,self.angular_vel):
                #cte=self.get_cte()
                cte=0
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,self.linear_vel,self.angular_vel,self.dbw_enabled,cte)
            if self.dbw_enabled:
                self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()

    def pose_cb(self,msg):
        self.pose=msg
        self.frame_id=msg.header.frame_id

    def waypoints_cb(self,msg):
        self.waypoints=msg

    def velocity_cb(self,msg):
        self.current_vel=msg.twist.linear.x
        #rospy.logdebug("Current Velocity {}".format(self.current_vel))

    def dbw_enabled_cb(self,msg):
        #rospy.logdebug("IN DBW CALL BACK")
        self.dbw_enabled=msg.data;
        #rospy.logdebug("DBW Status=%s",self.dbw_enabled)

    def twist_cb(self,msg):
        self.linear_vel=msg.twist.linear.x
        self.angular_vel=msg.twist.angular.z
        self.twist_cmd=msg

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
