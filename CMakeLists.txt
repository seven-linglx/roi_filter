cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME roi_filter)
project(${PROJECT_NAME})

#set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)
find_file(Eigen REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITION})

set(SOURCE_FILES main.cpp roi_filter.cpp roi_filter.h)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${CERES_LIBRARIES} gflags)
