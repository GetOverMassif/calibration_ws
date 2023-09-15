cmake_minimum_required(VERSION 3.5)
project(imu_calibration)

set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

#Vis3D
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( 
    ${OPENGL_INCLUDE_DIRS}  
    ${GLUT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

set(IMU_TK_VIS_EXTRA_SOURCES
    ./src/imu_tk/vis_extra/gl_camera.cpp 
    ./src/imu_tk/vis_extra/opengl_3d_scene.cpp
)
set(IMU_TK_VIS_EXTRA_HEADERS
    ./include/imu_tk/vis_extra/opengl_3d_scene.h
)

qt4_wrap_cpp(IMU_TK_VIS_EXTRA_HEADERS_MOC ${IMU_TK_VIS_EXTRA_HEADERS})

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

aux_source_directory( ./src/imu_tk IMU_TK_CORE_SRC )

set(IMU_TK_SRC ${IMU_TK_CORE_SRC} ${IMU_TK_VIS_EXTRA_HEADERS_MOC} ${IMU_TK_VIS_EXTRA_SOURCES})

add_library(imu_tk ${IMU_TK_SRC})
set_target_properties(imu_tk PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set (IMU_TK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "imu_tk include directories")
set (IMU_TK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib CACHE STRING "imu_tk libraries directories")
set (IMU_TK_LIBS imu_tk ${CERES_LIBRARIES} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}
     CACHE STRING "imu_tk libraries")

add_executable(imu_calibration src/imu_calibration.cpp)

ament_target_dependencies(imu_calibration rclcpp)

target_link_libraries(imu_calibration ${IMU_TK_LIBS})
set_target_properties(imu_calibration PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

install(TARGETS
    imu_calibration
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
