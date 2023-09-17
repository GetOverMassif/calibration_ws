cmake_minimum_required(VERSION 3.5)
project(imu_extrinstic_calibration)

set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)  
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/include
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)


add_executable(imu_extr_calibration src/imu_extr_calibration.cpp)

ament_target_dependencies(imu_extr_calibration rclcpp)

install(TARGETS
    imu_extr_calibration
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
