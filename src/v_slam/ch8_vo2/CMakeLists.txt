add_executable(optical_flow optical_flow.cc)
target_link_libraries(optical_flow 
${PROJECT_NAME}.common
        ${third_party_libs})



add_executable(direct_method direct_method.cc)
target_link_libraries(direct_method 
${PROJECT_NAME}.common
${third_party_libs})