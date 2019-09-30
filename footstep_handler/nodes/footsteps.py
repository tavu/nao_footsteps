#!/usr/bin/env python


import rospy
from footstep_handler import FootstepHandler
#import footstepHandler
import time
import strings
from geometry_msgs.msg import PoseWithCovarianceStamped ,PoseStamped, Pose
from nav_msgs.msg import Path
import sys
from std_msgs.msg import  Header

import utils
footstepHandler = None
maxSteps = 8
pathPubl = None

def pose_callback(pose_with_cov):    
    global footstepHandler
    start = pose_with_cov.pose.pose 
    footstepHandler.setStart(start)
    return 

def goal_callback(goal):
    global footstepHandler
    print(goal)
    #start = utils.pose_to_pose2D(pose_with_cov.pose.pose )

def publish_steps(steps):        
    global pathPubl
    path = Path()
    for step in steps.footsteps:
        pose =  PoseStamped()
        pose.header = Header();
        pose.header.frame_id = strings.odom_frame
        pose.header.stamp = rospy.Time.now()
        #pose.header.seq = seq
        pose.pose = utils.pose2D_to_pose(step.pose)
        path.poses.append(pose)
    
    path.header = Header()
    path.header.frame_id = strings.odom_frame
    path.header.stamp = rospy.Time.now()
    #print(path)
    pathPubl.publish(path)
    return
        
rospy.init_node('path_handler', anonymous=True,disable_signals=False)
#global pathPubl

footstepHandler = FootstepHandler()

rospy.Subscriber(strings.pose_topic, PoseWithCovarianceStamped, pose_callback)
rospy.Subscriber(strings.goal_topic, PoseStamped, goal_callback)

pathPubl = rospy.Publisher(strings.pub_path_topic,  Path, queue_size=10)

goal_pose = Pose()
goal_pose.position.x= 1
goal_pose.position.y=  0
goal_pose.position.z=  0.0

goal_pose.orientation.x=0
goal_pose.orientation.y=0    
goal_pose.orientation.z=-0.0154884964884
goal_pose.orientation.w=0.999880046044

goal = utils.pose_to_pose2D(goal_pose)


time.sleep(3)

footstepHandler.setGoal(goal)
#hasFInished = False

rate = rospy.Rate(2)
while not rospy.is_shutdown():
    if footstepHandler.askPath():
        steps = footstepHandler.getSteps(maxSteps)
        publish_steps(steps)
        if steps is not None:
            pass
            #footstepHandler.sendSteps(steps)
        if footstepHandler.hasFInished():
            break
    rate.sleep()
    #sys.exit()

   

rospy.spin()
 
