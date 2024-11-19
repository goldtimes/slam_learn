add_executable(triangulate
triangulate.cc 
)
target_link_libraries(triangulate 
${PROJECT_NAME}.common
${third_party_libs})