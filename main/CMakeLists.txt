cmake_minimum_required (VERSION 2.6)

link_libraries(reforges)
find_package (Threads)
add_executable(reforges_test reforges.cpp)
target_link_libraries (reforges_test ${CMAKE_THREAD_LIBS_INIT})