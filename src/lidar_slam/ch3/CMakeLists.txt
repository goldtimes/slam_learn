add_executable(test_imu_integration
        test/test_imu_integration.cc
        )

target_link_libraries(test_imu_integration
        glog gflags ${PROJECT_NAME}.common
        )
