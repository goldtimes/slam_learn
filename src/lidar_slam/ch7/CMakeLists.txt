add_library(${PROJECT_NAME}.ch7
    icp_3d.cc
)

target_link_libraries(${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.common
        ${third_party_libs}
)


add_executable(test_icp
        test/test_icp.cc
        )

target_link_libraries(test_icp
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
)