add_executable(hessian_nullspace_test 
    hessian_nullspace_test.cc 
)
target_link_libraries(hessian_nullspace_test 
${PROJECT_NAME}.common
${third_party_libs})