cmake_minimum_required(VERSION 3.10.2)
project(rosbaz)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW) #3.12.0 `find_package()`` uses ``<PackageName>_ROOT`` variables.
endif()

option(REGRESSION_TESTS "Build and execute regression tests" OFF)

set(CMAKE_CXX_STANDARD 11)
set(ROSBAZ_CXX_FLAGS
  -Wall
  -Wextra
  -Wcast-qual
  -Wconversion-null
  -Wmissing-declarations
  -Woverlength-strings
  -Wpointer-arith
  -Wunused-local-typedefs
  -Wunused-result
  -Wvarargs
  -Wvla
  -Wwrite-strings
  -Wold-style-cast
  -Wshadow
  -Wnon-virtual-dtor
  -Wcast-align
  -Wunused
  -Woverloaded-virtual
  -Wpedantic
  -Wconversion
  # Google style does not use unsigned integers though STL containers
  # have unsigned types.
  -Wno-sign-compare
)

add_compile_options(
  "$<$<COMPILE_LANGUAGE:CXX>:${ROSBAZ_CXX_FLAGS}>"
)

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

file(GLOB _lib_sources CONFIGURE_DEPENDS "src/*.cpp" "src/**/*.cpp")

add_library(${PROJECT_NAME}
  ${_lib_sources}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    azure_storage_cpplite::azure_storage_cpplite
    ${catkin_LIBRARIES}
)

file(GLOB _app_sources CONFIGURE_DEPENDS "app/*.cpp" "app/**/*.cpp")

add_executable(${PROJECT_NAME}_app
  ${_app_sources}
)
target_link_libraries(${PROJECT_NAME}_app PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_app PROPERTIES OUTPUT_NAME "rosbaz")

set_target_properties(${PROJECT_NAME}_app
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}"
)

if (CATKIN_ENABLE_TESTING)
  file(GLOB _test_sources CONFIGURE_DEPENDS "test/*.cpp")
  catkin_add_gtest(test_${PROJECT_NAME} ${_test_sources})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)

  if (REGRESSION_TESTS)
    include(ExternalProject)
    ExternalProject_Add(bag-file
      URL               https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b0-2014-07-11-10-58-16.bag
      URL_HASH          MD5=7ed841ca72cb82adcf4a3fc6b85cac5a
      DOWNLOAD_NO_EXTRACT TRUE
      CONFIGURE_COMMAND ""
      INSTALL_COMMAND   ""
      DOWNLOAD_DIR      ${CMAKE_CURRENT_BINARY_DIR}
      BUILD_COMMAND     rosbag decompress ${CMAKE_CURRENT_BINARY_DIR}/b0-2014-07-11-10-58-16.bag
    )

    file(GLOB _regression_test_sources CONFIGURE_DEPENDS "regression_test/*.cpp")
    catkin_add_gtest(regression_test_${PROJECT_NAME} ${_regression_test_sources})
    target_link_libraries(regression_test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main)

    add_dependencies(regression_test_${PROJECT_NAME} bag-file)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_app
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)