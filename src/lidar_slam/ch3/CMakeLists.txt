add_library(${PROJECT_NAME}.ch3
        static_imu_init.cc
        utm_convert.cc
        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
)

target_link_libraries(${PROJECT_NAME}.ch3
        glog gflags ${PROJECT_NAME}.common
        )

        add_executable(test_imu_integration
        test/test_imu_integration.cc
        )

target_link_libraries(test_imu_integration
        glog gflags ${PROJECT_NAME}.common
        )
add_executable(test_eskf_gins
        test/test_eskf_gins.cc
        )

target_link_libraries(test_eskf_gins
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.ch3
        )
