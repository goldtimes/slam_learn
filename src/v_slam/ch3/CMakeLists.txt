add_executable(CurveFitting 
    CurveFitting.cc
    backend/vertex.cc
    backend/edge.cc
    backend/problem.cc
)
target_link_libraries(CurveFitting 
${PROJECT_NAME}.common
${third_party_libs})