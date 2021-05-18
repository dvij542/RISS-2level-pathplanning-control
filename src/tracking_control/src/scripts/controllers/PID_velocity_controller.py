#!/usr/bin/env python
'''
PID_velocity_controller
This code implements a simple PID controller for path tracking.
Author : Het Shah
'''
import time
import rospy
import math
import json
import xmltodict
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import thread
from prius_msgs.msg import Control
from tf2_msgs.msg import TFMessage
import numpy as np
# import matplotlib.pyplot as plt

# Node name       - controls
# Published topic  - pid_output (Twist)
# Subscribed topic - base_pose_ground_truth , cmd_vel, cmd_delta

gear_stat = "F"
# target omega
tar_delta = 0
active_vel = 0  # current velocity of robot
error_sum = 0
prev_error = 0
error_diff = 0
output = 0
wheelbase = 1.958  # in meters
radius = 0  # radius of path
steering_angle = 0  # steering angle in degrees
kp = 1000.0  # proportional gain
ki = 1.5  # integral gain
kd = 1.8  # differential gain
acc_thershold = 0  # threshold for acceleration
brake_threshold = 20  # threshold for brake
global pub
global tar_vel
global tar_omega
global tar_delta


def prius_pub(data):
    '''
    publishes the velocity and steering angle
    published on topic : ackermann_cmd_topic
    '''
    global prius_vel
    prius_vel = Control()

    if(data.linear.x > 0):
        prius_vel.throttle = data.linear.x / 100
        prius_vel.brake = 0
        print("acc")
        print(prius_vel.throttle)

    if(data.linear.x < 0):
        prius_vel.brake = -data.linear.x / 100
        prius_vel.throttle = 0
        print("brake")
        print(prius_vel.brake)

    prius_vel.steer = data.angular.z / 30
    # print "steering:", prius_vel.steer

    pub.publish(prius_vel)

iter0 = 0
gt_steering = []
init_steering_x = 0
init_steering_y = 0
init_steering_z = 0
init_steering_w = 0

def convert_xyzw_to_rpy(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def callback_tf(data):
    global iter0
    global gt_steering
    global times
    global pub
    global init_steering_x,init_steering_y,init_steering_z,init_steering_w
    for tf_data in data.transforms :
        if tf_data.header.frame_id == "chassis" and tf_data.child_frame_id == "fr_axle" :
            if iter0==0 :
                init_steering_x = tf_data.transform.rotation.x
                init_steering_y = tf_data.transform.rotation.y
                init_steering_z = tf_data.transform.rotation.z
                init_steering_w = tf_data.transform.rotation.w
                
            iter0 = iter0+1
            if iter0 >= 100 :
                print(gt_steering)
                np.savetxt("front_right_wheel_0.5.csv",np.array(gt_steering))
                print("Saved")
                exit()
            x = tf_data.transform.rotation.x #- init_steering_x
            y = tf_data.transform.rotation.y #- init_steering_y
            z = tf_data.transform.rotation.z #- init_steering_z
            w = tf_data.transform.rotation.w #- init_steering_w
            xp = tf_data.transform.translation.x
            yp = tf_data.transform.translation.y
            zp = tf_data.transform.translation.z
            r,p,yaw = convert_xyzw_to_rpy(x,y,z,w)
            print(str(tf_data.header.stamp))
            t = float(str(tf_data.header.stamp))/1e+09
            gt_steering.append([t,yaw*180/3.14])
            test_cmd = Control()
            test_cmd.brake = 0
            test_cmd.throttle = 0
            test_cmd.steer = -0.5
            pub.publish(test_cmd)



def callback_feedback(data):
    '''
    Applies PID to velcity input from odom readings and publishes.
    :params data [Odometry]
    :params output [Twist]
    :params plot [Twist] 
    '''
    global active_vel
    global tar_vel
    global tar_delta
    global tar_omega
    global wheelbase
    global error_sum
    global error
    global error_diff
    global output
    global i
    global flag
    global kp
    global ki
    global kd
    global pub
    global prev_error
    global gear_stat
    global acc_thershold
    global brake_threshold
    global act_velocity
    # conversion of odometry readings from quarternion to euler
    siny = +2.0 * (data.pose.pose.orientation.w *
                   data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x *
                   data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
                         data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z *
                         data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy)

    last_recorded_vel = (data.twist.twist.linear.x * math.cos(yaw) +
                         data.twist.twist.linear.y * math.sin(yaw))

    active_vel = last_recorded_vel

    plot = Twist()
    output = Twist()
    # applying PID on the Velocity
    error = tar_vel - active_vel
    error_sum += error
    error_diff = error - prev_error
    prev_error = error
    if error == 0:
        print("e")
        if tar_vel == 0:
            output.linear.x = 0
        else:
            output.linear.x = output.linear.x - 5

    if error > 0.01:
        print("e1")
        output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
    if error < -0.01:
        print("e2")
        output.linear.x = ((kp * error + ki * error_sum + kd * error_diff) -
                           brake_threshold)

    plot.linear.x = tar_vel
    plot.linear.y = active_vel
    plot.linear.z = tar_vel - active_vel  # error term

    # thresholding the forward velocity within -100 to 100
    if output.linear.x > 100:
        output.linear.x = 100
    if output.linear.x < -100:
        output.linear.x = -100

    # Thresholding the steering angle between 30 degrees and -30 degrees
    output.angular.z = min(30.0, max(-30.0, tar_delta))
    rospy.loginfo("linear velocity : %f", plot.linear.y)
    rospy.loginfo("target linear velocity : %f", plot.linear.x)
    rospy.loginfo("delta : %f", output.angular.z)
    # publish the msg
    prius_pub(output)
    pub1.publish(plot)


def callback_cmd_vel(data):
    '''
    Subscribes from cmd_vel and gives target velocity
    :params data [Twist]
    :params tar_vel [float]
    '''
    global tar_vel
    tar_vel = data.linear.x

    # smoothing the velocity and reducing it at greater turning angles.


def callback_delta(data):
    '''
    Subscribes from cmd_delta and returns target steering angle.
    :params data [Twist]
    :params tar_delta [Float] 
    '''
    global tar_delta
    tar_delta = data.angular.z


def start():
    global pub
    global pub1
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/prius')
    print(ackermann_cmd_topic)
    print("YES changed")
    test_cmd = Control()
    test_cmd.brake = 0
    test_cmd.throttle = 0
    test_cmd.steer = -1.57
    rospy.init_node('controls', anonymous=True)
    pub = rospy.Publisher(ackermann_cmd_topic, Control, queue_size=10)
    # response = []
    # for i in range(100) :
    #     pub.publish(test_cmd)
    #     # time.sleep(0.05)
    #     robot_description = rospy.get_param('/robot_description')
    #     robot_dict = xmltodict.parse(robot_description)
    #     gt_steering = float(robot_dict[u'robot'][u'joint'][1]['origin'][u'@rpy'].split(' ')[0])
    #     print(gt_steering)
    #     response.append(gt_steering)
    #     print("Published")

    # print(response)
    # # plt.plot(response)
    # plt.show()
    # print(ackermann_cmd_topic)
    # pub1 = rospy.Publisher('plot', Twist, queue_size=10)
    # rospy.Subscriber("cmd_vel_frenet", Twist, callback_cmd_vel)
    # rospy.Subscriber("cmd_delta", Twist, callback_delta)
    # rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
    rospy.Subscriber("/tf", TFMessage, callback_tf)
    
    rospy.spin()


if __name__ == '__main__':
    start()
