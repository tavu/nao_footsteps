cmake_minimum_required(VERSION 2.8.3)
project(footstep_planner)

find_package(catkin REQUIRED COMPONENTS actionlib angles gridmap_2d humanoid_nav_msgs map_server roscpp rospy tf visualization_msgs sbpl)
link_directories(${CATKIN_PACKAGE_LIB_DESTINATION})
find_package(OpenCV REQUIRED)

#find_package(sbpl REQUIRED)
find_package(PkgConfig REQUIRED)
# pkg_check_modules(SBPL REQUIRED sbpl)
# include_directories(${SBPL_INCLUDE_DIRS})
# link_directories(${SBPL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sbpl
)

MESSAGE (sdf ${SBPL_INCLUDE_DIRS})
MESSAGE (sdf s ${SBPL_LIBRARY_DIRS})
MESSAGE (sdf s ${catkin_LIBRARIES})

set(FOOTSTEP_PLANNER_FILES src/FootstepPlanner.cpp
    src/FootstepNavigation.cpp
    src/FootstepPlannerNode.cpp
    src/FootstepPlannerEnvironment.cpp 
    src/Footstep.cpp
    src/PlanningState.cpp
    src/Heuristic.cpp 
    src/helper.cpp
    src/PathCostHeuristic.cpp
    src/StepCostHeuristic.cpp
    src/EuclidianStepCost2.cpp
    src/PlanningStateChangeQuery.cpp
    src/State.cpp
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

set(SBPL_LIBRARIES "/home/tavu/catkin_ws/devel/lib")
add_library(${PROJECT_NAME} ${FOOTSTEP_PLANNER_FILES})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

add_executable(footstep_planner_node src/footstep_planner.cpp)
target_link_libraries(footstep_planner_node ${PROJECT_NAME} )

add_executable(footstep_planner_walls src/footstep_planner_walls.cpp)
target_link_libraries(footstep_planner_walls ${PROJECT_NAME}  ${OpenCV_LIBS})

add_executable(footstep_navigation_node src/footstep_navigation.cpp)
target_link_libraries(footstep_navigation_node ${PROJECT_NAME} )

# install
install(TARGETS ${PROJECT_NAME} footstep_planner_node footstep_planner_walls 
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS footstep_navigation_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
