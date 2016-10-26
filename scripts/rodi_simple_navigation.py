#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped

def se2_from_6dof(pose):
    q = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    yaw = tf.transformations.euler_from_quaternion(q)[2]
    return pose.position.x, pose.position.y, yaw

class RodiSimpleNavigation(object):

    def __init__(self, linear_velocity, angular_velocity):
        self.listener = tf.TransformListener()
        self.rodi_vel_pub = rospy.Publisher('cmd_vel',
                                        Twist,
                                        queue_size=1)
        rospy.Subscriber('goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.new_goal = False
        self.pose = None
        self.goal = None
        self.goal_reached = False
        self.heading_reached = False
        self.goal_received = False
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

    def has_reached_goal(self):
        return math.fabsf(self.pose[0] - self.goal[0]) < 0.1 and math.fabsf(self.pose[1] - self.goal[1]) < 0.1

    def has_reached_heading(self):
        return math.fabsf(self.pose[2] - self.goal[2]) < 0.01

    def goal_callback(self, goal):
        self.goal = se2_from_6dof(goal.pose)
        rospy.loginfo("Goal set to x={0:.2f}, y={1:.2f}, th={2:.2f}".format(*self.goal))
        self.goal_received = True
        self.goal_reached = False

    def pose_callback(self, pose):
        self.pose = se2_from_6dof(pose.pose)
        if self.goal_received and not self.goal_reached:
            
            angle_to_goal = math.atan2(self.goal[1] - self.pose[1],
                                       self.goal[0] - self.pose[0]) - self.pose[2]
            distance_to_goal = math.sqrt((self.goal[1] - self.pose[1]) ** 2
                                         + (self.goal[0] - self.pose[0]) ** 2)
            rospy.logdebug("Angle to goal = {0:.2f} | Distance to goal = {1:.2f}".format(angle_to_goal, distance_to_goal))
            print angle_to_goal, distance_to_goal
            cmd_vel = Twist()
            
            if(math.fabs(angle_to_goal) < 0.01 and distance_to_goal < 0.01):
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.goal_reached = True
                rospy.loginfo("Goal reached: x=(0:.2f}, y={1:.2f}, th={2:.2f}".format(*self.pose))
            else:
                cmd_vel.linear.x = self.linear_velocity * distance_to_goal
                cmd_vel.angular.z = self.angular_velocity * angle_to_goal
            
            self.rodi_vel_pub.publish(cmd_vel)

    #def run(self):
    #    if self.new_goal:
    #        pass
    #    rate = rospy.Rate(10.0)
    #    while not rospy.is_shutdown():
    #        try:
    #            (trans, rot) = self.listener.lookupTransform('/goal',
    #                                                         '/base_link',
    #                                                         rospy.Time(0))
    #        except (tf.LookupException,
    #                tf.ConnectivityException,
    #                tf.ExtrapolationException):
    #            continue
#
#            if self.goal_received and not self.goal_reached:
#                angular = self.angular_velocity.gc * math.atan2(trans[1], trans[0])
#                linear = self.linear_velocity.gc * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
#                cmd = Twist()
#                if(math.fabs(angular) < 0.01 and linear < 0.01):
#                    cmd.linear.x = 0.0
#                    cmd.angular.z = 0.0
#                    self.goal_reached = True
#                    rospy.info("Goal reached: x=(0:.2f}, y={1:.2f}, th={2:.2f}".format(self.pose.x, self.pose.y, self.pose.theta))
#                else:
#                    cmd.linear.x = linear
#                    cmd.angular.z = angular
#
#                self.rodi_vel.publish(cmd)
        #rate.sleep()


if __name__ == '__main__':
    rospy.init_node('rodi_simple_navigator')
    linear_velocity = rospy.get_param('~linear_velocity', 0.5)
    angular_velocity = rospy.get_param('~angular_velocity', 2)
    rodi_navigation = RodiSimpleNavigation(linear_velocity, angular_velocity)
    rospy.spin()
