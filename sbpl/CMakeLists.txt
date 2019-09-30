cmake_minimum_required(VERSION 2.8)
PROJECT( sbpl )
# find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
   INCLUDE_DIRS "src/include"
   LIBRARIES sbpl
   CATKIN_DEPENDS roscpp
)

# version (e.g. for packaging)
set(SBPL_MAJOR_VERSION 1)
set(SBPL_MINOR_VERSION 2)
set(SBPL_PATCH_VERSION 0)
set(SBPL_VERSION ${SBPL_MAJOR_VERSION}.${SBPL_MINOR_VERSION}.${SBPL_PATCH_VERSION})

set(CMAKE_BUILD_TYPE Debug)

add_definitions( -DDEBUG=1 )
add_definitions( -DROS)

include_directories(src/include ${catkin_INCLUDE_DIRS})

add_library(sbpl SHARED
  src/discrete_space_information/environment_nav2D.cpp
  src/discrete_space_information/environment_navxythetalat.cpp
  src/discrete_space_information/environment_navxythetamlevlat.cpp
  src/discrete_space_information/environment_nav2Duu.cpp
  src/discrete_space_information/environment_XXX.cpp
  src/discrete_space_information/environment_robarm.cpp
  src/planners/araplanner.cpp
  src/planners/adplanner.cpp
  src/planners/viplanner.cpp
  src/planners/ppcpplanner.cpp
  src/planners/ANAplanner.cpp
  src/planners/rstarplanner.cpp
  src/planners/lazyARA.cpp
  src/utils/heap.cpp
  src/utils/mdp.cpp
  src/utils/utils.cpp
  src/utils/2Dgridsearch.cpp
  src/utils/config.cpp	
  )

#   install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE)
  
# set(SBPL_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
# set(SBPL_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib")
  
# support for pkg-config
# set(PKG_NAME "sbpl")
# set(PKG_DESC "The Search Based Planning Library")
# set(PKG_SBPL_LIBS "-lsbpl")
# set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/sbpl.pc.in")
# set(pkg_conf_file "${PROJECT_BINARY_DIR}/InstallFiles/sbpl.pc")
# configure_file("${pkg_conf_file_in}" "${pkg_conf_file}" @ONLY)
# install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)

# # support for cmake-config:
# configure_file(sbpl-config.cmake.in
#   "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config.cmake" @ONLY)
# configure_file(sbpl-config-version.cmake.in
#   "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config-version.cmake" @ONLY)
# install(FILES
#   "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config.cmake"
#   "${PROJECT_BINARY_DIR}/InstallFiles/sbpl-config-version.cmake"
# DESTINATION share/sbpl/)
# 
# install(DIRECTORY src/include/sbpl DESTINATION include
# 	FILES_MATCHING PATTERN "*.h")
# install(TARGETS sbpl
# 	LIBRARY DESTINATION lib)

add_executable(test_sbpl src/test/main.cpp)
target_link_libraries(test_sbpl sbpl ${catkin_LIBRARIES})

add_executable(test_adjacency_list src/test/test_adjacency_list.cpp)
target_link_libraries(test_adjacency_list sbpl ${catkin_LIBRARIES})

# target_link_libraries(sbpl ${catkin_LIBRARIES})

