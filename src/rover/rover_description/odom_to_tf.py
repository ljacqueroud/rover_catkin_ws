#!/usr/bin/env python

import rospy
from std_msgs.msg import String , Float64

from geometry_msgs.msg import Twist

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg

from nav_msgs.msg import Odometry





def callback(odometry):
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "CHASSIS"
    t.transform.translation.x = odometry.pose.pose.position.x
    t.transform.translation.y = odometry.pose.pose.position.y
    t.transform.translation.z = odometry.pose.pose.position.z

    t.transform.rotation.x = odometry.pose.pose.orientation.x
    t.transform.rotation.y = odometry.pose.pose.orientation.y
    t.transform.rotation.z = odometry.pose.pose.orientation.z
    t.transform.rotation.w = odometry.pose.pose.orientation.w

    br.sendTransform(t)



def odom_transform():



    rospy.init_node('odom_to_tf', anonymous=True)

    sub = rospy.Subscriber("/odom", Odometry , callback)

    rospy.spin()



if __name__ == '__main__':
    try:
        odom_transform()
    except rospy.ROSInterruptException:
        pass
