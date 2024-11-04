find_package(GTest REQUIRED)
add_library(${PROJECT_NAME}.ch4
        imu_preintegration.cc
        g2o_preintegration.cc
        gins_pre_integ.cc
        )
# test
ADD_EXECUTABLE(test_preintegration test/test_preintegration.cc)
ADD_TEST(test_preintegration test_preintegration)
#  解决gmock 编译错误问题

target_link_libraries(test_preintegration
        ${GTEST_BOTH_LIBRARIES}
        pthread 
        glog 
        gflags 
        ${PROJECT_NAME}.ch4 
        ${PROJECT_NAME}.ch3 
        ${PROJECT_NAME}.common 
        gomp
)

add_executable(test_gins_pre_integ test/test_gins_pre_integ.cc)
target_link_libraries(test_gins_pre_integ
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch4
        ${g2o_libs}
        gomp
        )