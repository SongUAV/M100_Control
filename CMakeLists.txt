cmake_minimum_required(VERSION 2.6)
project(reforges)

#add_definitions(-pg) #"-fopenmp)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME reforges)
include(cmake/pods.cmake)

file(GLOB SOURCE_FILES "src/*.c*")
include_directories(reforges. /opt/local/include)
add_library(reforges ${SOURCE_FILES})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(reforges ${OpenCV_LIBS}) #-pg) #-fopenmp)
pods_use_pkg_config_packages(reforges eigen3)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(reforges -L/opt/local/lib/) # MacPorts special treatment...
else (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  pods_use_pkg_config_packages(reforges libv4l2)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

pods_install_libraries(reforges)

file(GLOB header_files "includes/*.h")
pods_install_headers(${header_files} DESTINATION includes/)

pods_install_pkg_config_file(reforges
    LIBS -lreforges
    REQUIRES eigen3 opencv2
    VERSION 1.0.0)

add_subdirectory(main)
