cmake_minimum_required(VERSION 3.15.3)
project(rosbaz)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rosbag
)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(azure_storage_cpplite REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rosbag
  DEPENDS azure_storage_cpplite
)

file(GLOB_RECURSE _lib_sources CONFIGURE_DEPENDS "src" "*.cpp")

add_library(${PROJECT_NAME}
  ${_lib_sources}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    azure_storage_cpplite::azure_storage_cpplite
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_app
  "app/rosbaz.cpp"
)
target_link_libraries(${PROJECT_NAME}_app PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_app PROPERTIES OUTPUT_NAME "rosbaz")

set_target_properties(${PROJECT_NAME}_app
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_app
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)