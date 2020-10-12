cmake_minimum_required(VERSION 3.5)
project(deepstream_interpipe)

set(CMAKE_CXX_STANDARD 14)

#include_directories(/usr/include/gstreamer-1.0/ /usr/include/glib-2.0/ /usr/lib/x86_64-linux-gnu/glib-2.0/include/ /opt/nvidia/deepstream/deepstream-5.0/sources/includes/)
# Gstreamer
find_package(PkgConfig)

pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4
        gstreamer-sdp-1.0>=1.4
        gstreamer-video-1.0>=1.4
        gstreamer-app-1.0>=1.4)
message(STATUS "Find Gstreamer include at ${GST_INCLUDE_DIRS}")
message(STATUS "Find Gstreamer libraries: ${GST_LIBRARIES}")

include_directories(${GST_INCLUDE_DIRS} /opt/nvidia/deepstream/deepstream-5.0/sources/includes/)
link_directories(${GST_LIBRARIES} /opt/nvidia/deepstream/deepstream-5.0/lib/)

add_executable(deepstream_interpipe deepstream_interpipe.cpp)
target_link_libraries(deepstream_interpipe ${GST_LIBRARIES} libnvdsgst_meta.so libnvds_meta.so)
