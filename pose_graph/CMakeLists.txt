cmake_minimum_required(VERSION 2.8.3)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    camera_model
    cv_bridge
    roslib
)

find_package(GTSAM REQUIRED)

find_package(OpenCV)

find_package(Ceres REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})


add_executable(pose_graph
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
)

# add_executable(Trial src/Trial.cpp)


target_link_libraries(pose_graph ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} gtsam)
message(${OpenCV_LIBS})
# target_link_libraries(Trial ${catkin_LIBRARIES}  ${CERES_LIBRARIES} ${OpenCV_LIBS} gtsam)

# message("message" ${gtsam})
# message("catkin_lib  ${catkin_LIBRARIES}")
