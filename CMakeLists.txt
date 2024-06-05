cmake_minimum_required(VERSION 3.5)
project(ESKF)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

add_definitions("-DCATKIN_ENABLE_TESTING=0")


set(CMAKE_PREFIX_PATH "/opt/ros/noetic/share/catkin" ${CMAKE_PREFIX_PATH})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")
set(glog_DIR "/usr/local/lib/cmake/glog")
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

find_package(PythonLibs 3.8 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LIBRARIES})
include_directories(/home/czy/matplotlib-cpp)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        )
include_directories(
            # include
              ${catkin_INCLUDE_DIRS}
            )
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
#find_package(Glog REQUIRED)
#include_directories(${Glog_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(gflags REQUIRED)


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/build/devel/include)

add_executable(run_eskf
        "src/run_eskf.cc"
        )
add_executable(kf
        "src/kf/kf.cc"
        )
add_library(ku ${PROJECT_SOURCE_DIR}/src/io_utils.cc ${PROJECT_SOURCE_DIR}/src/tranmerc.cc 
${PROJECT_SOURCE_DIR}/src/utm_convert.cc ${PROJECT_SOURCE_DIR}/src/utm.cc ${PROJECT_SOURCE_DIR}/src/static_imu_init.cc
)
target_link_libraries(run_eskf ku Eigen3::Eigen ${catkin_LIBRARIES} ${OpenCV_LIBS} glog gflags fmt)
target_link_libraries(kf ${PYTHON_LIBRARIES})