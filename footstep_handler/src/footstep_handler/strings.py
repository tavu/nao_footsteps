#TOPICS
goal_action = 'move_base'
goal_topic = 'goal' 
pose_topic = 'amcl_pose'
path_topic = '/footstep_planner/path'
start_topic = "/footstep_planner/start"
pub_path_topic = 'steps'
joint_topic = 'joint_states'
odom_topic = 'SERoW/odom'
time_topic = 'nao_timer'
#FRAMES
map_frame='map'
odom_frame='odom'
l_sole_frame = 'l_sole'
r_sole_frame = 'r_sole'


#SERVICES
walk_action="/footsteps_execution" 
path_service="plan_footsteps"

path_service_feet = 'plan_footsteps_feet'

#LEGS
right = 0
left = 1