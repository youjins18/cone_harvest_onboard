cmake_minimum_required(VERSION 3.16)
project(task_space_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- ROS2 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------- RBDL (Rigid Body Dynamics Library, 시스템 설치 필요) ----------
# 사용자가 사전에 RBDL을 BUILD_ADDON_URDFREADER=ON 으로 빌드/설치해야 함.
#   예: colcon build --cmake-args -DRBDL_DIR=/path/to/rbdl-install
find_package(Eigen3 REQUIRED)
set(RBDL_DIR "$ENV{HOME}/rbdl-install" CACHE PATH "RBDL 설치 경로")
find_path(RBDL_INCLUDE_DIR NAMES rbdl/rbdl.h
  HINTS "${RBDL_DIR}/include" /usr/local/include /usr/include)
find_library(RBDL_LIBRARY NAMES rbdl
  HINTS "${RBDL_DIR}/lib" /usr/local/lib /usr/lib)
find_path(RBDL_URDFREADER_INCLUDE_DIR NAMES rbdl/addons/urdfreader/urdfreader.h
  HINTS "${RBDL_DIR}/include" /usr/local/include /usr/include)
find_library(RBDL_URDFREADER_LIBRARY NAMES rbdl_urdfreader
  HINTS "${RBDL_DIR}/lib" /usr/local/lib /usr/lib)
if(NOT RBDL_INCLUDE_DIR OR NOT RBDL_LIBRARY OR NOT RBDL_URDFREADER_INCLUDE_DIR OR NOT RBDL_URDFREADER_LIBRARY)
  message(FATAL_ERROR
    "RBDL(+URDF reader addon)을 찾을 수 없습니다. BUILD_ADDON_URDFREADER=ON으로 빌드/설치 후 "
    "-DRBDL_DIR=/path/to/rbdl-install 로 지정하세요.")
endif()
message(STATUS "RBDL include: ${RBDL_INCLUDE_DIR}")
message(STATUS "RBDL lib    : ${RBDL_LIBRARY}")
message(STATUS "RBDL urdfreader include: ${RBDL_URDFREADER_INCLUDE_DIR}")
message(STATUS "RBDL urdfreader lib    : ${RBDL_URDFREADER_LIBRARY}")

# ---------- Task-space kinematics 라이브러리 (ROS 비종속, RBDL 래퍼) ----------
add_library(task_space_kinematics src/task_space_kinematics.cpp)
target_include_directories(task_space_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${RBDL_INCLUDE_DIR} ${RBDL_URDFREADER_INCLUDE_DIR})
target_link_libraries(task_space_kinematics ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} Eigen3::Eigen)

# ---------- Task-space control 노드 (목표 pose 구독 -> joint 궤적 발행) ----------
add_executable(task_space_controller_node src/task_space_controller_node.cpp)
target_include_directories(task_space_controller_node PRIVATE
  ${RBDL_INCLUDE_DIR} ${RBDL_URDFREADER_INCLUDE_DIR})
target_link_libraries(task_space_controller_node
  task_space_kinematics ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} Eigen3::Eigen)
ament_target_dependencies(task_space_controller_node
  rclcpp sensor_msgs geometry_msgs ament_index_cpp)
set_target_properties(task_space_controller_node PROPERTIES INSTALL_RPATH "${RBDL_DIR}/lib")
install(TARGETS task_space_controller_node DESTINATION lib/${PROJECT_NAME})

# ---------- 설치 ----------
install(TARGETS task_space_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
