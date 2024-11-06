add_library(${PROJECT_NAME}.ch9
        frontend.cc
        keyframe.cc
        optimization.cc
        loopclosure.cc
        )

target_link_libraries(${PROJECT_NAME}.ch9
        ${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch4
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.tools
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )
add_executable(run_frontend test/run_frontend.cc)
target_link_libraries(run_frontend
                ${PROJECT_NAME}.ch9
                ${third_party_libs}
                )
add_executable(run_mapping test/run_mapping.cc)
target_link_libraries(run_mapping
                        ${PROJECT_NAME}.ch9
                        ${third_party_libs}
                        )
        