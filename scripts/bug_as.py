#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from tf import transformations
from std_srvs.srv import *
import time

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
pose_ = Pose()
desired_position_ = Point()
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'done']
state_ = 0
# 0 - go to point
# 1 - wall following
# 2 - done
# 3 - canceled
# callbacks


def clbk_odom(msg):
    global position_, yaw_, pose_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
    
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    
    
def planning(goal):
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pose_
    change_state(0)
    rate = rospy.Rate(20)
    success = True
    
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    
    
    feedback = assignment_2_2023.msg.PlanningFeedback()
    result = assignment_2_2023.msg.PlanningResult()
    
    while not rospy.is_shutdown():
        err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) +
                        pow(desired_position_.x - position_.x, 2))
        if act_s.is_preempt_requested():
            rospy.loginfo("Goal was preeempted")
            feedback.stat = "Target cancelled!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            act_s.set_preempted()
            success = False
            change_state(2)
            done()
            break
        elif err_pos < 0.5:
            change_state(2)
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            break       
        elif regions_ == None:
            continue
        
        elif state_ == 0:
            feedback.stat = "State 0: go to point"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            if regions_['front'] < 0.2:
                change_state(1)
        elif state_ == 1:
            feedback.stat = "State 1: avoid obstacle"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                change_state(0)
        elif state_== 2:
            break
            
            
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)
    
    

def main():
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pub

    rospy.init_node('bug0')
    
    desired_position_.x = 0.0
    desired_position_.y = 1.0
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    srv_client_go_to_point_ = rospy.ServiceProxy(
        '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    act_s = actionlib.SimpleActionServer('/reaching_goal', assignment_2_2023.msg.PlanningAction, planning, auto_start=False)
    act_s.start()
   
    # initialize going to the point
    

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == "__main__":
    main()
