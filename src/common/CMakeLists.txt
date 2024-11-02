add_library(${PROJECT_NAME}.common
        io_utils.cc
        timer/timer.cc
        # global_flags.cc
        rosbag_io.cc
        g2o_types.cc
        # point_cloud_utils.cc
        )

target_link_libraries(${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )


set(slam_learn_messages
        # monitor_msgs_generate_messages
        # velodyne_msgs_generate_messages
        livox_ros_driver_generate_messages
        )

# add_dependencies(${PROJECT_NAME}.common)