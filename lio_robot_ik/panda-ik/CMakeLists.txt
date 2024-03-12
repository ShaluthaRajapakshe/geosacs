cmake_minimum_required(VERSION 3.4)
project(panda_ik)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  realtime_tools
  message_runtime
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geosacs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    realtime_tools
    roscpp
    tf2 
    tf2_ros
)


set(IK_LIB ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libpanda_ik.so)

include(ExternalProject)
ExternalProject_Add(
  panda_ik_rust
  PREFIX ${CMAKE_CURRENT_SOURCE_DIR}
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
  BUILD_COMMAND cargo build --release 
  BUILD_ALWAYS TRUE
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/target/release/libpanda_ik.so ${IK_LIB}
  UPDATE_COMMAND ""
  LOG_BUILD ON
)

add_library(panda_ik_rust_lib SHARED IMPORTED)
set_target_properties(panda_ik_rust_lib PROPERTIES IMPORTED_LOCATION ${IK_LIB})


add_executable(panda_ik
    cpp_src/main.cpp
)

add_dependencies(
  panda_ik
  panda_ik_rust
  panda_ik_rust_lib
)

target_link_libraries(panda_ik PUBLIC
  ${catkin_LIBRARIES}
  panda_ik_rust_lib
)

target_include_directories(panda_ik SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(panda_ik PUBLIC
  include
)

install(TARGETS panda_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY
    urdfs/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
)
