#!/usr/bin/env python
import rospy
import tf2_ros
import actionlib
from nav_msgs.msg import *
from humanoid_nav_msgs.msg import *

import math

from humanoid_nav_msgs.srv import *
from geometry_msgs.msg import PoseWithCovarianceStamped ,PoseStamped, Pose, Pose2D

from std_msgs.msg import  Header
import tf2_geometry_msgs

from stepQueue import StepQueue
import strings, utils
import copy


import tf
from error import Error

#import quaternion

#import almath_foot_clip
#import almath

class FootstepHandler(object):
    _tf_buffer = None
    _pathSrvFeet = None
    _tf_listener = None

    
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10000.0)) #tf buffer length
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._stepQueue = StepQueue()
        
        rospy.wait_for_service(strings.path_service_feet)
        self._pathSrvFeet = rospy.ServiceProxy(strings.path_service_feet, PlanFootstepsBetweenFeet)        
        self._separation = 0.105
        self._eps = 3.0
        
        self._start_left = None
        self._start_right = None
        self._goal_left = None
        self._goal_right = None        
        return
       
    def getStartPos(self):
        return [self._start_left, self._start_right]
    
    def getGoalPos(self):
        return [self._goal_left, self._goal_right]
    
    def clear(self):        
        self._stepQueue.clear()
        self._eps = 5.0
     
    def getFootFromRobotPose(self, robot_pose, leg):
        shift_x = - math.sin(robot_pose.theta) * self._separation / 2.0;
        shift_y =   math.cos(robot_pose.theta) * self._separation / 2.0;
        sign = -1.0
        if leg == strings.left:
            sign = 1.0
        
        ret = Pose2D()
        ret.x = robot_pose.x + sign * shift_x
        ret.y = robot_pose.y + sign * shift_y
        ret.theta = robot_pose.theta
        return ret
        
    def getFootPoseFromTf(self, foot_frame,tr_frame):        
        foot_pose = PoseStamped()
        foot_pose.header.frame_id = foot_frame        
        foot_pose.pose.position.x = 0
        foot_pose.pose.position.y = 0
        foot_pose.pose.position.z = 0
        foot_pose.pose.orientation.x = 0
        foot_pose.pose.orientation.y = 0
        foot_pose.pose.orientation.z = 0
        foot_pose.pose.orientation.w = 1
        if foot_frame == tr_frame:
            return foot_pose
        pose_transformed = self.delay_transform(foot_pose, tr_frame)
        return pose_transformed.pose    

    def getEps(self):
        return self._eps

    def async_plan(self, p_time):
        rospy.wait_for_service(strings.path_service_feet)
        try:
            resp = self._pathSrvFeet(self._start_left, self._start_right, self._goal_left, self._goal_right,p_time,False,self._eps)
            #resp = self._pathSrvFeet(msg)p_time
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return None
        if resp.result == True:
            self._eps = resp.final_eps
        return resp

    def plan(self, goal,p_time, until_first_solution):  
        self._stepQueue.clear()
        self._start_left = StepTarget()
        self._start_left.leg = strings.left
        start_pose_l = self.getFootPoseFromTf( strings.l_sole_frame, strings.map_frame)
        self._start_left.pose = utils.pose_to_pose2D(start_pose_l)
        
        self._start_right = StepTarget()
        self._start_right.leg = strings.right
        start_pose_r = self.getFootPoseFromTf( strings.r_sole_frame, strings.map_frame)
        self._start_right.pose = utils.pose_to_pose2D(start_pose_r)

        self._goal_left = StepTarget()
        self._goal_left.leg = strings.left
        self._goal_left.pose = self.getFootFromRobotPose(goal, strings.left)
        
        self._goal_right = StepTarget()
        self._goal_right.leg = strings.right        
        self._goal_right.pose = self.getFootFromRobotPose(goal, strings.right)
        
        #msg = PlanFootstepsBetweenFeet()
        #msg.start_left = self._start_left
        #msg.start_right = self._start_right
        #msg.goal_left = self._goal_left
        #msg.goal_right = self._goal_right
        #msg.planning_time = p_time
        #msg.until_first_solution = True
        try:
            resp = self._pathSrvFeet(self._start_left, self._start_right, self._goal_left, self._goal_right,p_time,until_first_solution,self._eps)
            #resp = self._pathSrvFeet(msg)p_time
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return Error.SERVICE_ERR

        if resp.result == True:
            rospy.loginfo("Planning succeeded with %d steps, path costs: %f, eps: %f" % (len(resp.footsteps), resp.costs, resp.final_eps))
            self._eps = resp.final_eps
            self._stepQueue.addSteps(resp.footsteps)
            return Error.OK
        else:
            rospy.logerr("Service call failed")            
            return Error.PLANER_ERR
      
    def getSteps(self, maxSteps):
        footsteps = []
        if self._stepQueue.isFInished():
            return None       
        i = 1
        step = self._stepQueue.nextStep()
        while (i < maxSteps) and (not self._stepQueue.isFInished() ):
            step = self._stepQueue.nextStep()
            step_tr = self.transformSteps(step)
            if step_tr is None:
                return None
            footsteps.append(step_tr)
            i = i + 1
        return footsteps
    
    def stepsNum(self):
        return self._stepQueue.size()

    def hasSteps(self):
        if self._stepQueue.isFInished():
            return False
        return True
        
    def delay_transform(self,obj,frame):     
        pose_transformed = None
        tota_delay = 0
        sleep_time = 0.1
        max_sleep = 2
        while  ( not rospy.is_shutdown() ) and (pose_transformed is None)  and (tota_delay < max_sleep):
            try:
                pose_transformed = self._tf_buffer.transform(obj, frame)
            except Exception as e:                
                rospy.logerr(str(e) )
                rospy.sleep(sleep_time)            
                tota_delay = tota_delay + sleep_time         
        #rospy.loginfo("tf transformation from %s to %s with delay %f." % ( obj.header.frame_id, frame, tota_delay ) )
        return pose_transformed
   
    def setFinalStep(self,footsteps):        
        last_step = footsteps[-1]
        final_step = copy.deepcopy(last_step)
        final_step_pose = None
        seperation = self._separation
        leg_frame = ' '
                
        shift_x = - math.sin(last_step.pose.theta) * self._separation
        shift_y =   math.cos(last_step.pose.theta) * self._separation        
        if last_step.leg  == strings.left:
            sign = -1.0
            final_step.leg = strings.right
        else:
            sign = 1.0
            final_step.leg = strings.left
        
        final_step.pose = Pose2D()
        final_step.pose.x = last_step.pose.x + sign * shift_x
        final_step.pose.y = last_step.pose.y + sign * shift_y
        final_step.pose.theta = last_step.pose.theta
        footsteps.append(final_step)
        return final_step   
    
    def transformSteps(self, step, frame_id = strings.map_frame ):
        ret = copy.deepcopy(step)
        step_pose = PoseStamped()        
        step_pose.header.frame_id = frame_id
        step_pose.pose = utils.pose2D_to_pose(step.pose)
        
        pose_transformed = self.delay_transform(step_pose, strings.odom_frame)
        if pose_transformed is None:
            return None        

        ret.pose = utils.pose_to_pose2D(pose_transformed.pose)
        return ret    
