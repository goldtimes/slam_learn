# 包含cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#sophus 只依赖头文件
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
 
# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIRS})

#spdlog
find_package(spdlog REQUIRED)

#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

#g2o
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o)
set(g2o_libs
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_stuff.so    
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_core.so    
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_dense.so    
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_csparse.so    
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_csparse_extension.so    
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_sba.so 
    ${CSPARSE_LIBRARY}
    ${CHOLMOD_LIBRARY}   
)

#ceres
find_package(Ceres REQUIRED)

#gtsam
find_package(GTSAM REQUIRED QUIET)
include_directories(${GTSAM_INCLUDE_DIR})

#Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#Boost
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time timer serialization)
include_directories(${BOOST_INCLUDE_DIRS})
# ros
set(ROS_PACKAGES
    roscpp 
    rospy
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions
    geometry_msgs
    nav_msgs
    tf
)
find_package(catkin QUIET COMPONENTS
    ${ROS_PACKAGES}
) 

set(third_party_libs
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${g2o_libs}
    ${CERES_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    gtsam
    glog
    gflags
    tbb
    spdlog::spdlog_header_only
)