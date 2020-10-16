cmake_minimum_required(VERSION 3.10)
project(deepstream_gst_metadata)

set(CMAKE_CXX_STANDARD 14)

# Gstreamer
find_package(PkgConfig)

pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4
        gstreamer-sdp-1.0>=1.4
        gstreamer-video-1.0>=1.4
        gstreamer-app-1.0>=1.4)
message(STATUS "Find Gstreamer include at ${GST_INCLUDE_DIRS}")
message(STATUS "Find Gstreamer libraries: ${GST_LIBRARIES}")

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "Find OpenCV include at ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Find OpenCV libraries: ${OpenCV_LIBRARIES}")

# cudawarping
set(CUDAWARPING_INCLUDE /home/lihan/depends/opencv_contrib/modules/cudawarping/include)

include_directories(${GST_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CUDAWARPING_INCLUDE} /opt/nvidia/deepstream/deepstream-5.0/sources/includes/)
link_directories(/opt/nvidia/deepstream/deepstream-5.0/lib/)

add_executable(deepstream_gst_metadata main.cpp)
target_link_libraries(deepstream_gst_metadata ${GST_LIBRARIES} ${OpenCV_LIBRARIES} libnvdsgst_meta.so libnvds_meta.so libnvbufsurface.so)