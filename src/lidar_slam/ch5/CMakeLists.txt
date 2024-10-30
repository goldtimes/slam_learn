add_executable(pcd_to_bird_eye pcd_to_bird_eye.cc)
target_link_libraries(pcd_to_bird_eye
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
)

add_executable(scan_to_range_image scan_to_range_image.cc)
target_link_libraries(scan_to_range_image
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
)