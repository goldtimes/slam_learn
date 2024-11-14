add_executable(ikd_tree_demo test/ikd_tree_demo.cc ikd_Tree.cc)
target_link_libraries(ikd_tree_demo ${third_party_libs})

# add_executable(ikd_tree_async_demo examples/ikd_Tree_Async_demo.cpp ikd-Tree/ikd_Tree.cpp)
# target_link_libraries(ikd_tree_async_demo ${PCL_LIBRARIES})

# add_executable(ikd_tree_Search_demo examples/ikd_Tree_Search_demo.cpp ikd-Tree/ikd_Tree.cpp)
# target_link_libraries(ikd_tree_Search_demo ${PCL_LIBRARIES})