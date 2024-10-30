add_library(${PROJECT_NAME}.ch4
        imu_preintegration.cc
        )
# test
ADD_EXECUTABLE(test_preintegration test/test_preintegration.cc)
ADD_TEST(test_preintegration test_preintegration)
target_link_libraries(test_preintegration
    gtest pthread glog gflags ${PROJECT_NAME}.ch4 ${PROJECT_NAME}.ch3 ${PROJECT_NAME}.common
)