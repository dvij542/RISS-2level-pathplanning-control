#!/usr/bin/env python
'''
This is the implementation of pure pursuit controller for path tracking
Link to reference paper: https://www.ri.cmu.edu/pub_files/2009/2
/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

Authors: Adarsh Patnaik, Anand Jhunjhunwala

'''

import rospy
import math
import json
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from prius_msgs.msg import Control
from sensor_msgs.msg import Range

# node name: path_tracking
# Publish Topic: cmd_delta
# Subscribe Topic: base_pose_ground_truth, astroid_path

global sonar_d_l 
global sonar_d_r
safe_d = 3
is_safe_l =1
is_safe_r =1

max_vel = 20.0  # maximum linear velocity
global steer
k = 0.5  # constant for relating look ahead distance and velocity
wheelbase = 1.983  # wheel base for the vehicle
d_lookahead = 0.1  # look ahead distance to calculate target point on path
global n
global ep_max
global ep_sum
global ep_avg
global q

print("start")
q = 0
n = 0
ep_avg = 0
ep_sum = 0
ep_max = 0


def callback_feedback(data):
    '''
    Assigns the position of the robot to global variables from odometry.
    :param x_bot [float]
    :param y_bot [float]
    :param yaw [float]
    :param vel [float]
    '''
    global x_bot
    global y_bot
    global yaw
    global vel

    x_bot = data.pose.pose.position.x
    y_bot = data.pose.pose.position.y
    # quarternion to euler conversion
    siny = +2.0 * (data.pose.pose.orientation.w *
                   data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x *
                   data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
                         data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z *
                         data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy)  # yaw in radians
    # vel = data.twist.twist.linear.x * math.cos(yaw)
    # + data.twist.twist.linear.y * math.sin(yaw)
    # printing the odometry readings

    # print 'x of car:', x_bot
    # print 'y of car:', y_bot
    # print 'angle of car:', yaw
    # print 'vel of car:', data.twist.twist.linear.x,

    data.twist.twist.linear.y,
    data.twist.twist.linear.z
    # print 'c'


def dist(a, x, y):
    '''
    Calculates distance between two points.
    :param a [float]
    :param x [float]
    :param y [float]
    '''
    # calculate distance
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5


def path_length_distance(a, b):
    return (((a.pose.position.x - b.pose.position.x)**2) + ((a.pose.position.y - b.pose.position.y)**2))**0.5


def calc_path_length(data):
    global path_length
    path_length = []

    for i in range(len(data.poses)):
        if i == 0:
            path_length.append(0)

        else:
            path_length.append(path_length[i - 1] + path_length_distance(data.poses[i], data.poses[i - 1]))

def callback_sonar_r(data):
    print "SONAAR RIGHT DATA ARRIVED\n"
    print "Sonar right data : ",data.range,"\n"
    global is_safe_r
    sonar_d_r = data.range

    if (sonar_d_r < safe_d):
        steer_angle = 45
        is_safe_r=0
        print "***obstacle at RIGHT***\n"

    cmd = Twist()
    if(is_safe_r==0):
        cmd.angular.z = min(30, max(-30, steer_angle * 180 / math.pi))
        pub1.publish(cmd)

def callback_sonar_l(data):
    print "SONAAR LEFT DATA ARRIVED\n"
    print "Sonar left data : ",data.range,"\n"
    global is_safe_l
    sonar_d_l = data.range
    # print "sonar_d_l: ",sonar_d_l,"\n"

    if (sonar_d_l < safe_d):
        steer_angle = -45
        is_safe_l=0
        print "***obstacle at LEFT***\n"
    else: 
        is_safe_l=1
    # cmd = Twist()
    # if(is_safe_l==0):
    #     cmd.angular.z = min(30, max(-30, steer_angle * 180 / math.pi))
    #     pub1.publish(cmd)

def callback_path(data):
    '''
    calculates target path point and the steering angle
    :param data [Path]
    :param ep [float]
    :param cp [int]
    '''
    global ep  # min distance
    global cp  # index of closest point
    global ep_max
    global ep_sum
    global ep_avg
    global n
    global cp1
    global path_length

    cross_err = Twist()
    x_p = data
    # calculate minimum distance
    calc_path_length(x_p)

    distances = []
    for i in range(len(x_p.poses)):
        a = x_p.poses[i]
        distances += [dist(a, x_bot, y_bot)]
    ep = min(distances)
    ep1 = ep

    if (ep > ep_max):
        ep_max = ep

    n = n + 1
    ep_sum = ep_sum + ep
    ep_avg = ep_sum / n

    cp = distances.index(ep)
    cp1 = cp
    cross2 = [(x_bot - data.poses[cp1].pose.position.x),
              (y_bot - data.poses[cp1].pose.position.y)]
    cross = [math.cos(yaw), math.sin(yaw)]
    cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
    if (cross_prod > 0):
        ep1 = -ep1

    print 'ep_sum: ', ep_sum
    print 'ep_avg: ', ep_avg
    cross_err.linear.x = ep1
    cross_err.angular.x = ep_max
    cross_err.angular.y = ep_avg

    print 'old index:', cp
    # calculate index of target point on path
    cmd = Twist()
    cmd1 = Twist()
    prius_vel = Control()
    L = 0
    Lf = k * max_vel + d_lookahead

    while Lf > L and (cp + 1) < len(x_p.poses):
        dx = data.poses[cp + 1].pose.position.x - \
            data.poses[cp].pose.position.x
        dy = data.poses[cp + 1].pose.position.y - \
            data.poses[cp].pose.position.y
        L += math.sqrt(dx ** 2 + dy ** 2)
        cp = cp + 1
    print len(x_p.poses)
    print 'new index is:', cp

    goal_point = [x_p.poses[cp].pose.position.x,
                  x_p.poses[cp].pose.position.y]
    print 'current goal is:', goal_point
    error = [goal_point[0] - x_bot, goal_point[1] - y_bot]
    print error

    if(is_safe_l and is_safe_r):
        steer_angle = pure_pursuit(goal_point)

    siny = +2.0 * (x_p.poses[cp].pose.orientation.w *
                   x_p.poses[cp].pose.orientation.z +
                   x_p.poses[cp].pose.orientation.x *
                   x_p.poses[cp].pose.orientation.y)

    cosy = +1.0 - 2.0 * (x_p.poses[cp].pose.orientation.y *
                         x_p.poses[cp].pose.orientation.y +
                         x_p.poses[cp].pose.orientation.z *
                         x_p.poses[cp].pose.orientation.z)

    steer_path = math.atan2(siny, cosy)
    steer_err = (yaw - steer_path)
    cross_err.linear.y = (-1) * (yaw - steer_path)

    print "steer_angle :", steer_angle * 180 / math.pi
    cmd.angular.z = min(30, max(-30, steer_angle * 180 / math.pi))
    cmd.linear.y = math.sqrt(error[0]**2 + error[1]**2)
    print 'omega:', cmd.angular.z
    cross_err.linear.z = path_length[cp]

    #r = rospy.Rate(100)
    # while not rospy.is_shutdown():
    pub1.publish(cmd)
    pub2.publish(cross_err)
    #   r.sleep()

    print "cmd published"

    # print (ep)
    print x_p.poses[cp].pose.orientation

    is_safe=1


def pure_pursuit(goal_point):
    '''
    Calculates the steering angle required for path tracking
    :params goal_point [float,float] goal point coordinates
    :params Delta [float] steering angle in radians 
    '''
    tx = goal_point[0]
    ty = goal_point[1]
    print 'yaw:', yaw
    # measuring the slope of path point
    print 'slope:', math.atan2(ty - y_bot, tx - x_bot)
    # measuring heading angle
    alpha = math.atan2(ty - y_bot, tx - x_bot) - yaw
    print 'alpha:', alpha
    Lf = k * max_vel + d_lookahead
    # measuring the steering angle using pure pursuit controller
    Delta = math.atan2(2.0 * wheelbase * math.sin(alpha) / Lf, 1)
    print 'Delta:', Delta
    return Delta


def start():
    global pub1
    global pub2
    rospy.init_node('path_tracking', anonymous=True)
    rate = rospy.Rate(10)
    pub2 = rospy.Publisher('cross_track_error', Twist, queue_size=100)
    pub1 = rospy.Publisher('cmd_delta', Twist, queue_size=100)
    rospy.Subscriber("/prius/front_sonar/right_far_range",Range,callback_sonar_r)
    rospy.Subscriber("/prius/front_sonar/left_far_range",Range,callback_sonar_l)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
    rospy.Subscriber("astroid_path", Path, callback_path)
    
    rospy.spin()



if __name__ == '__main__':
    start()