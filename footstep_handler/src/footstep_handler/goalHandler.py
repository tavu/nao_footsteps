import rospy
import strings
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import  Header
from humanoid_nav_msgs.srv import *
#from humanoid_nav_msgs.msg import ExecFootstepsGoal
from humanoid_nav_msgs.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction
import utils
from footstepHandler import FootstepHandler
from error import Error
import actionlib
from actionlib_msgs.msg import GoalStatus
from footstepThr import FootstepThr
from humanoid_nav_msgs.srv import *
from footstep_handler.msg import time_msg

class GoalHandler(object): 
    def __init__(self, maxSteps):
        self._footstepHandler = FootstepHandler()
        self._maxSteps = maxSteps
        self._stepTime = 0.30
        self._current_goal = None
        self._robotPose = None
        self._pathPubl = rospy.Publisher(strings.pub_path_topic,  Path, queue_size=10)
        self._timePubl = rospy.Publisher(strings.time_topic,  time_msg, queue_size=10)
        self._walkClient = actionlib.SimpleActionClient(strings.walk_action, ExecFootstepsAction)        
        rospy.Subscriber(strings.pose_topic, PoseWithCovarianceStamped, self._poseCallback)
        self._acc_x = 0.35
        self._acc_y = 0.2
        self._acc_theta = np.pi/3
        self._canceled = False
        self._new_goal = None
        self._server = actionlib.SimpleActionServer(strings.goal_action , MoveBaseAction, self.execute, False)
        self._server.start()
        self._thr = None
        self._max_planning_time = 15.0
        self._has_async_plan = True

      
        
    def _poseCallback(self, pose_with_cov):
        self._robotPose = pose_with_cov.pose.pose
        
    def hasFinished(self):
        if self._current_goal is None:
            return True
        if not self._footstepHandler.hasSteps():
            return True
        pose = utils.pose_to_pose2D(self._robotPose)
        x = abs(pose.x - self._current_goal.x)
        print("x distance: %f" % x)
        if x > self._acc_x:
            return False
        y = abs(pose.y - self._current_goal.y)
        if y > self._acc_y:
            return False
        theta = abs( self.anglediff2(pose.theta , self._current_goal.theta) )
        if theta > self._acc_theta:
            return False
        return True        
        
    def wrapToPi(self, angle):
		while angle > np.pi:
			angle -= 2.0 * np.pi
		while angle < - np.pi:
			angle += 2.0 * np.pi
		return angle

    def anglediff2(self, a1, a2):
        return self.wrapToPi(self.wrapToPi(a1 + np.pi - a2) - np.pi)

    def maxSteps(self):
        return self._maxSteps
    
    def setMaxSteps(self, maxSteps):
        self._maxSteps = maxSteps
    
    def cancel(self):
        if self._current_goal is not None:
            self._canceled = True

    def setGoal(self, goal):
        if self._current_goal is not None:
            self._new_goal = utils.pose_to_pose2D(goal.pose)
            #cancle possible previous goal
            self.cancel()
        else:
            self._current_goal = utils.pose_to_pose2D(goal.pose)
    
    def goal(self):
        return self._current_goal
    
    def clear(self):
        self._current_goal = self._new_goal
        self._new_goal = None
        self._canceled = False
        self._footstepHandler.clear()

    #def waitForGoal(self):
        #r = rospy.Rate(1)
        #rospy.loginfo('Waiting for goal')
        #while not rospy.is_shutdown():
            #if _goal is not None:
                #self.handleGoal()
                #self._footstepHandler.clear()
                #self._goal = None
                #rospy.loginfo('Waiting for goal')
            #else:
                #r.sleep()

    def publish_steps(self, steps):
        path = Path()
        for step in steps:
            pose =  PoseStamped()
            pose.header = Header()
            pose.header.frame_id = strings.odom_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose = utils.pose2D_to_pose(step.pose)
            path.poses.append(pose)
        
        path.header = Header()
        path.header.frame_id = strings.odom_frame    
        path.header.stamp = rospy.Time.now()
        self._pathPubl.publish(path)    
        return
        
    def sendStepsToNao(self, footsteps):
        stepsGoal = ExecFootstepsGoal()
        stepsGoal.footsteps = footsteps
        self._walkClient.send_goal(stepsGoal)        
        #results = self._walkClient.wait_for_result()
        #if not results:
        #    return Error.WALK_ERR
        #return Error.OK
    
    def waitForRobotToStop(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.robotIsStopped():
                break
            r.sleep()
        print("Robot stoped")
        #rospy.sleep(2)
    
    def execute(self,goal):
        self._current_goal = utils.pose_to_pose2D(goal.target_pose.pose)
        self._canceled = False
        status = self.handleGoal()
        results = False
        if status == Error.OK:
            results = True
            rospy.loginfo('Robot reached destination')
        elif status == Error.CANCELED:
            self._server.set_preempted()
            rospy.logerr("Goal cancelled.")
        else:
            rospy.logerr("Robot did not reach goal")
        self.clear()
        self._server.set_succeeded(results)
        
    def handleGoal(self):
        while not rospy.is_shutdown():
            status = Error.OK
            if self._has_async_plan:
                status = self._footstepHandler.plan(self._current_goal,self._max_planning_time,True)
            else:
                status = self._footstepHandler.plan(self._current_goal,self._max_planning_time,False)
            if status != Error.OK:
                return status
            #return status
            steps = self._footstepHandler.getSteps(self._maxSteps-1)
            if steps is None:
                rospy.logerr('Tf error')
                return TF_ERR
            self.publish_steps(steps)
            if self._footstepHandler.hasSteps():
               final_step = self._footstepHandler.setFinalStep(steps)
            #append dummy step
            steps.append(final_step)
            #print(self._footstepHandler.getEps())
            self.sendStepsToNao(steps)
            self._has_async_plan = True
            timeMsg = time_msg()
            timeMsg.header = Header()
            timeMsg.header.stamp = rospy.Time.now()
            timeMsg.goal = self._current_goal
            timeMsg.remaining_steps = self._footstepHandler.stepsNum()
            timeMsg.remaining_time = self._footstepHandler.stepsNum() * self._stepTime
            self._timePubl.publish(timeMsg)
            if self._footstepHandler.getEps() > 1.0 and len(steps) * self._stepTime > 1:
                print("Start async planning: ")
                asyc_resp = self._footstepHandler.async_plan(len(steps) * self._stepTime -1)
                if asyc_resp is not None and asyc_resp.result == True:
                    print("Async planningterminated with eps:%f" % asyc_resp.final_eps)
                else:
                    print("Async planningterminated with an error")
                    self._has_async_plan = False
            self._walkClient.wait_for_result()
            results = self._walkClient.get_state()
            print(results)
            if results != GoalStatus.SUCCEEDED:
               err = Error.WALK_ERR
            else:
	           err = Error.OK
     
            #err = self.sendStepsToNao(steps)



            #if self._thr is not None:
            #    self._thr.join()
            #    self._thr = None
            if err != Error.OK:
                print('walking error')
                #self.clear()
                return err
            #self.waitForRobotToStop()
            if self.hasFinished():
                self._footstepHandler.clear()
                return Error.OK
            if self._server.is_preempt_requested():
                #self.clear()
                return Error.CANCELED
            self._footstepHandler.clear()

