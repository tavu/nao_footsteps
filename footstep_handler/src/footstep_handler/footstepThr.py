import threading 
import rospy
import strings
#from nav_msgs.msg import *
#from humanoid_nav_msgs.msg import *
from humanoid_nav_msgs.srv import *
class FootstepThr(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        
        self._pathSrvFeet = rospy.ServiceProxy(strings.path_service_feet, PlanFootstepsBetweenFeet)
        self._time = 10
        self._start_left = None
        self._start_right = None
        self._goal_left = None
        self._goal_right = None
    
    def setStartPos(self, start):
        self._start_left = start[0]
        self._start_right = start[1]

    def setGoalPos(self, goal):
        self._goal_left = goal[0]
        self._goal_right = goal[1]
        
    def setMaxTime(self,time):
        self._time = time
    def run(self):
        print("this is an new thred")
        #d = rospy.Duration(10, 0)
        #rospy.sleep(d)
        rospy.wait_for_service(strings.path_service_feet)
        try:
            resp = self._pathSrvFeet(self._start_left, self._start_right, self._goal_left, self._goal_right,self._time,False)
        except rospy.ServiceException as exc:
            print("Thr error: " + str(exc))            
        print("New thred terminates")
