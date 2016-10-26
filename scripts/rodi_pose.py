#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('rodi_pose')

    listener = tf.TransformListener()

    rodi_pose = rospy.Publisher('pose', geometry_msgs.msg.PoseStamped, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (x, y, z), (qx, qy, qz, qw) = listener.lookupTransform('/rodi', '/world', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        rodi_pose.publish(pose)

        rate.sleep()
