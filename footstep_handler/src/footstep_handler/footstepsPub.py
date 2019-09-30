#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped ,PoseStamped, Pose2D
from std_msgs.msg import Header
import strings
import math
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import MarkerArray
import tf


pub_footsteps_topic = "footsteps_viz"

foot_origin_shift_x = 0.02
foot_origin_shift_y = 0.006

footsize_x = 0.16
footsize_y = 0.088
footsize_z = 0.015

#footsteps_x = [0.00, 0.06,-0.04, 0.00, 0.05, 0.01, 0.015, 0.04,-0.03, 0.06, 0.04,-0.02]
#footsteps_y = [0.16, 0.09, 0.09, 0.12, 0.14, 0.13, 0.100, 0.12, 0.12, 0.12, 0.10, 0.12]
#footsteps_theta = [0.00, 0.00, 0.00, 0.00, 0.00,-0.50, 0.500, 0.30, 0.50, 0.00, 0.00, 0.00]

footsteps_x =         [0.00,  0.03,  -0.02,  0.02,   0.01,  0.01,  0.0,  0.0,   -0.02, 0.03, 0.03, -0.02]
footsteps_y =         [0.12,  0.10,   0.10,   0.11,   0.12,  0.12, 0.12, 0.12,  0.12,  0.12, 0.10, 0.12]
footsteps_theta =   [0.00,  0.00,   0.00,   0.00,   -0.1,   0.1, 0.1,   -0.1,   0.1,   0.00, 0.00, 0.00]


dx = 0.2
x = 0
def footPoseToMarker(pose,leg):
    m = Marker()
    m.header=Header()
    m.header.frame_id = strings.map_frame
    m.header.stamp = rospy.Time.now()
    m.ns = ' '
    m.type = Marker.CUBE
    m.action = Marker.ADD
    cos_theta = math.cos(pose.theta)
    sin_theta = math.sin(pose.theta)
    
    x_shift = cos_theta * foot_origin_shift_x - sin_theta * foot_origin_shift_y
    if leg == strings.left:
        y_shift = sin_theta * foot_origin_shift_x + cos_theta * foot_origin_shift_y
    else:
        y_shift = sin_theta * foot_origin_shift_x - cos_theta * foot_origin_shift_y
   
    m.pose.position.x  = pose.x +  x_shift
    m.pose.position.y  = pose.y +  y_shift
    m.pose.position.z = footsize_z / 2.0
    q=tf.transformations.quaternion_from_euler(0.0, 0.0, pose.theta)
    m.pose.orientation.x = q[0]
    m.pose.orientation.y = q[1]
    m.pose.orientation.z = q[2]
    m.pose.orientation.w = q[3]
    
    m.scale.x = footsize_x
    m.scale.y = footsize_y 
    m.scale.z = footsize_z
     
    if leg == strings.right:  
        m.color.r = 0.0
        m.color.g = 1.0
    else:
        m.color.r = 1.0
        m.color.g = 0.0
  
    m.color.b = 0.0
    m.color.a = 0.6

    m.lifetime = rospy.Duration()
    return m

def getMarker(markers,i,id):
    p = Pose2D()
    p.x = x + i*dx
    p.y = 0
    p.theta = 0        
    m = footPoseToMarker(p, strings.right)
    markers.markers.append(m)
    m.id = id
    
    p1 = Pose2D()
    p1.x = footsteps_x[i] + p.x
    p1.y = footsteps_y[i]
    p1.theta = footsteps_theta[i]    
    m1 = footPoseToMarker(p1, strings.left)
    m1.id = id +1
    markers.markers.append(m1)

if __name__ == '__main__':
    rospy.init_node('viz_footsteps', anonymous=True)
    #rospy.Subscriber(strings.pose_topic, PoseWithCovarianceStamped, pose_callback)    
    pub = rospy.Publisher(pub_footsteps_topic, MarkerArray, queue_size=10)
    Hz = 1
    rate = rospy.Rate(Hz) # 10hz
    
    
    size = len(footsteps_x)
    print(size)
    while not rospy.is_shutdown():
        id = 0    
        markers = MarkerArray()
        for i in range(0, size):
            getMarker(markers,i,id)
            id = id +2
        pub.publish(markers)
        rate.sleep()
    rospy.spin()
    
    
