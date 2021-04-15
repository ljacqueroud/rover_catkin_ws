#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg       import Float64
from tf.transformations import euler_from_quaternion


if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pub_base_angle_correction = rospy.Publisher('/rover/transmission_rocker_left/command', Float64, queue_size=10)

    err_angle = Float64()
    old_err_angle = Float64()
    old_err_angle.data = 0

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try: # we look for the transformation from the latest -the 0 in rospy.Time(0)- CHASSIS to the ROVER_FRAME frame
            trans_1 = tfBuffer.lookup_transform('CHASSIS', 'ROVER_FRAME', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        try: # we look for the transformation from ROVER_FRAME to the ROCKER_LEFT frame
            trans_2 = tfBuffer.lookup_transform('ROVER_FRAME', 'ROCKER_LEFT', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        try: # we look for the transformation from ROVER_FRAME to the ROCKER_RIGHT frame
            trans_3 = tfBuffer.lookup_transform('ROVER_FRAME', 'ROCKER_RIGHT', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # get rotation between the frames expressed in Euler angles [rad]
        tf_rotation_data_fields = [trans_1.transform.rotation.x, trans_1.transform.rotation.y, trans_1.transform.rotation.z, trans_1.transform.rotation.w]
        (roll_1, pitch_1, yaw_1) = euler_from_quaternion(tf_rotation_data_fields)
        tf_rotation_data_fields = [trans_2.transform.rotation.x, trans_2.transform.rotation.y, trans_2.transform.rotation.z, trans_2.transform.rotation.w]
        (roll_2, pitch_2, yaw_2) = euler_from_quaternion(tf_rotation_data_fields)
        tf_rotation_data_fields = [trans_3.transform.rotation.x, trans_3.transform.rotation.y, trans_3.transform.rotation.z, trans_3.transform.rotation.w]
        (roll_3, pitch_3, yaw_3) = euler_from_quaternion(tf_rotation_data_fields)
        
        p_gain = 0.5  #p gain
        d_gain = 10   #d gain

        err_angle.data = pitch_1 + 0.5*(pitch_2+pitch_3)
        ctrl_cmd = p_gain*(err_angle.data) + d_gain*(err_angle.data - old_err_angle.data)*0.1

        pub_base_angle_correction.publish(ctrl_cmd)

        old_err_angle.data = err_angle.data   #store the current error value for the proportional term
        
        #print("pitch 1", pitch_1*180/3.14, "pitch 2", pitch_2*180/3.14, "pitch 3", pitch_3*180/3.14, "commanded action", ctrl_cmd*180/3.14)
        
        rate.sleep()