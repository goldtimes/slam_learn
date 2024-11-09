add_executable(orb_cv orb_cv.cc)
target_link_libraries(orb_cv 
${PROJECT_NAME}.common
        ${third_party_libs})



add_executable(pose_estimation_2d2d pose_estimation_2d2d.cc)
target_link_libraries(pose_estimation_2d2d 
${PROJECT_NAME}.common
${third_party_libs})

add_executable(triangulation triangulation.cc)
target_link_libraries(triangulation 
${PROJECT_NAME}.common
${third_party_libs})

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cc)
target_link_libraries(pose_estimation_3d2d
${PROJECT_NAME}.common
${third_party_libs})


add_executable(pose_estimation_3d3d pose_estimation_3d3d.cc)
target_link_libraries(pose_estimation_3d3d
${PROJECT_NAME}.common
${third_party_libs})
