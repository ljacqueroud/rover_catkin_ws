#!/usr/bin/env python

import rospy
import tf2_ros
from std_msgs.msg import String , Float64
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from numpy import linalg

# global variables, dont judge me
pub_r1 = rospy.Publisher('/rover/right_joint_effort_controller_1/command', Float64, queue_size=10)    #right rocker wheel
pub_r2 = rospy.Publisher('/rover/right_joint_effort_controller_2/command', Float64, queue_size=10)
pub_r3 = rospy.Publisher('/rover/right_joint_effort_controller_3/command', Float64, queue_size=10)

pub_l1 = rospy.Publisher('/rover/left_joint_effort_controller_1/command', Float64, queue_size=10)
pub_l2 = rospy.Publisher('/rover/left_joint_effort_controller_2/command', Float64, queue_size=10)
pub_l3 = rospy.Publisher('/rover/left_joint_effort_controller_3/command', Float64, queue_size=10)

rotation_l = Float64()
rotation_l.data = 0
rotation_l_mid = Float64()
rotation_l_mid.data = 0
rotation_l_rear = Float64()
rotation_l_rear.data = 0

rotation_r = Float64()
rotation_r.data = 0
rotation_r_mid = Float64()
rotation_r_mid.data = 0
rotation_r_rear = Float64()
rotation_r_rear.data = 0

tfBuffer = tf2_ros.Buffer()


def callback(twist):
    """
    components of Twist() :
    twist.linear.x
    twist.linear.y
    twist.linear.z
    twist.angular.x
    twist.angular.y
    twist.angular.z

    reference rover_base frame axes:
    x positive: forward
    y positive: left
    z positive: up
    
    TODO : add saturations at least on twist.angular.z (if not done by another control layer) 
           as too high values (roughly > 4) cause the lift of mid and rear wheels.
           Addtionally, two gain variables are defined to help and their values can be tuned
           (see lines 133-134)
    TODO : add saturations on max min wheel velocity

    """
    
    rospy.loginfo(twist)

    # look at the transformations between the base and wheels frames
    try:
        trans_l_1 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_LEFT_1', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Error: tf with left wheel 1 not found")
        pass
    try: 
        trans_l_2 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_LEFT_2', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Error: tf with left wheel 2 not found")
        pass
    try: 
        trans_l_3 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_LEFT_3', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Error: tf with left wheel 3 not found")
        pass
    try: 
        trans_r_1 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_RIGHT_1', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Error: tf with right wheel 1 not found")
        pass
    try: 
        trans_r_2 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_RIGHT_2', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Error: tf with right wheel 2 not found")
        pass
    try: 
        trans_r_3 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_RIGHT_3', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Error: tf with right wheel 3 not found")
        pass
    
    tf_lin_data_fields = [trans_l_1.transform.translation.x, trans_l_1.transform.translation.y, trans_l_1.transform.translation.z]
    l_b_cpt = linalg.norm(tf_lin_data_fields)
    tf_lin_data_fields = [trans_r_1.transform.translation.x, trans_r_1.transform.translation.y, trans_r_1.transform.translation.z]
    r_b_cpt = linalg.norm(tf_lin_data_fields)
    tf_lin_data_fields = [trans_l_2.transform.translation.x, trans_l_2.transform.translation.y, trans_l_2.transform.translation.z]
    l_mid_b_cpt = linalg.norm(tf_lin_data_fields)
    tf_lin_data_fields = [trans_r_2.transform.translation.x, trans_r_2.transform.translation.y, trans_r_2.transform.translation.z]
    r_mid_b_cpt = linalg.norm(tf_lin_data_fields)
    tf_lin_data_fields = [trans_l_3.transform.translation.x, trans_l_3.transform.translation.y, trans_l_3.transform.translation.z]
    l_rear_b_cpt = linalg.norm(tf_lin_data_fields)
    tf_lin_data_fields = [trans_r_3.transform.translation.x, trans_r_3.transform.translation.y, trans_r_3.transform.translation.z]
    r_rear_b_cpt = linalg.norm(tf_lin_data_fields)


    lin_vel = twist.linear.x
    ang_vel = twist.angular.z


    # compute individual wheel velocity values based on a kinematic model
    # the current kinematic model considers the geometrical distances
    # between the rover base_frame and the wheel-ground contact points only
    if lin_vel == 0 and ang_vel:
        # turn in place
        if ang_vel > 0: 
            # left turn
            rotation_r.data = - ang_vel*r_b_cpt
            rotation_l.data = - ang_vel*l_b_cpt
            rotation_r_mid.data = 0 #- ang_vel #*r_mid_b_cpt
            rotation_l_mid.data = 0 #- ang_vel #*l_mid_b_cpt
            rotation_r_rear.data = - ang_vel*r_rear_b_cpt
            rotation_l_rear.data = - ang_vel*l_rear_b_cpt
        else: 
            # right turn
            rotation_r.data = - ang_vel*r_b_cpt
            rotation_l.data = - ang_vel*l_b_cpt
            rotation_r_mid.data = 0 #- ang_vel #*r_mid_b_cpt
            rotation_l_mid.data = 0 #- ang_vel #*l_mid_b_cpt
            rotation_r_rear.data = - ang_vel*r_rear_b_cpt
            rotation_l_rear.data = - ang_vel*l_rear_b_cpt
    else:
        # tunable gains to prevent any wheel to lose the contact with the ground
        red_gain_lin = 0.7
        red_gain_ang = 0.5
        if ang_vel > 0:                                 
            # left turn
            rotation_r.data = -lin_vel*red_gain_lin - ang_vel*r_b_cpt*red_gain_ang
            rotation_l.data = lin_vel*red_gain_lin - ang_vel*l_b_cpt*red_gain_ang
            rotation_r_mid.data = -lin_vel*red_gain_lin - ang_vel*r_mid_b_cpt*red_gain_ang
            rotation_l_mid.data = lin_vel*red_gain_lin - ang_vel*l_mid_b_cpt*red_gain_ang
            rotation_r_rear.data = -lin_vel*red_gain_lin - ang_vel*r_rear_b_cpt*red_gain_ang
            rotation_l_rear.data = lin_vel*red_gain_lin - ang_vel*l_rear_b_cpt*red_gain_ang
        elif ang_vel < 0:                               
            # right turn
            rotation_r.data = -lin_vel*red_gain_lin - ang_vel*r_b_cpt*red_gain_ang
            rotation_l.data = lin_vel*red_gain_lin - ang_vel*l_b_cpt*red_gain_ang
            rotation_r_mid.data = -lin_vel*red_gain_lin - ang_vel*r_mid_b_cpt*red_gain_ang
            rotation_l_mid.data = lin_vel*red_gain_lin - ang_vel*l_mid_b_cpt*red_gain_ang
            rotation_r_rear.data = -lin_vel*red_gain_lin - ang_vel*r_rear_b_cpt*red_gain_ang
            rotation_l_rear.data = lin_vel*red_gain_lin - ang_vel*l_rear_b_cpt*red_gain_ang
        else:                                                   
            # straight motion
            rotation_r.data = -lin_vel 
            rotation_l.data = lin_vel 
            rotation_r_mid.data = -lin_vel 
            rotation_l_mid.data = lin_vel 
            rotation_r_rear.data = -lin_vel 
            rotation_l_rear.data = lin_vel 
    
    pub_r1.publish( rotation_r )
    pub_r2.publish( rotation_r_mid )
    pub_r3.publish( rotation_r_rear )

    pub_l1.publish( rotation_l )
    pub_l2.publish( rotation_l_mid )
    pub_l3.publish( rotation_l_rear )
    
    
    # previous implementation:
    # if twist.angular.z < 0:
    #     rotation_r.data = -twist.linear.x*2 + twist.angular.z*1
    #     rotation_l.data = twist.linear.x*2 - twist.angular.z*10

    #     rotation_r_mid.data = rotation_r.data*1.4
    #     rotation_l_mid.data = rotation_l.data*1
    #     rotation_r_rear.data = rotation_r.data*0.5
    #     rotation_l_rear.data = rotation_l.data*0.5

    # elif twist.angular.z > 0:
    #     rotation_l.data = twist.linear.x*2 + twist.angular.z*1
    #     rotation_r.data = -(twist.linear.x*2 + twist.angular.z*10)

    #     rotation_r_mid.data = rotation_r.data*1
    #     rotation_l_mid.data = rotation_l.data*1.4
    #     rotation_r_rear.data = rotation_r.data*0.5
    #     rotation_l_rear.data = rotation_l.data*0.5

    # else:
    #     rotation_r.data = -twist.linear.x*6
    #     rotation_l.data = twist.linear.x*6

    #     rotation_r_mid.data = rotation_r.data*1
    #     rotation_l_mid.data = rotation_l.data*1
    #     rotation_r_rear.data = rotation_r.data*1
    #     rotation_l_rear.data = rotation_l.data*1


def dummy_controller():

    rospy.init_node('twist_to_wheels', anonymous=True)

    # read the tf between suspension joints
    listener_tf = tf2_ros.TransformListener(tfBuffer)
    # cmd_vel msg contains longitudinal and angular velocities expressed with respect to the base_frame
    sub = rospy.Subscriber("/cmd_vel", Twist, callback)



    rospy.spin()


if __name__ == '__main__':
    try:
        dummy_controller()
    except rospy.ROSInterruptException:
        pass
