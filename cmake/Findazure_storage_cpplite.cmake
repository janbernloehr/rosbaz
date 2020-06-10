find_path(azure_storage_cpplite_INCLUDE_DIR
  NAMES storage_account.h
  HINTS $ENV{azure_storage_cpplite_ROOT} ${azure_storage_cpplite_ROOT_DIR} /usr/local/azure-storage-cpplite/
  PATH_SUFFIXES include
  DOC "Path to azure_storage_cpplite include directory.")

find_library(azure_storage_cpplite_LIBRARY
  NAMES azure-storage-lite
  HINTS $ENV{azure_storage_cpplite_ROOT} ${azure_storage_cpplite_ROOT_DIR} /usr/local/azure-storage-cpplite/
  PATH_SUFFIXES lib
  DOC "nvinfer library.")

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(azure_storage_cpplite
  REQUIRED_VARS
    azure_storage_cpplite_INCLUDE_DIR
    azure_storage_cpplite_LIBRARY
)

if (NOT azure_storage_cpplite_FOUND)
  unset(azure_storage_cpplite_INCLUDE_DIR CACHE)
  return()
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(uuid REQUIRED IMPORTED_TARGET uuid)

find_package(OpenSSL REQUIRED)
find_package(CURL REQUIRED)

if (NOT TARGET azure_storage_cpplite::azure_storage_cpplite)
  add_library(azure_storage_cpplite::azure_storage_cpplite STATIC IMPORTED GLOBAL)
  target_include_directories(azure_storage_cpplite::azure_storage_cpplite INTERFACE ${azure_storage_cpplite_INCLUDE_DIR})
  set_property(TARGET azure_storage_cpplite::azure_storage_cpplite
    PROPERTY
      IMPORTED_LOCATION "${azure_storage_cpplite_LIBRARY}")
  target_link_libraries(azure_storage_cpplite::azure_storage_cpplite INTERFACE ${CURL_LIBRARIES} PkgConfig::uuid OpenSSL::SSL OpenSSL::Crypto)

  get_filename_component(azure_storage_cpplite_LIBRARY_PATH "${azure_storage_cpplite_LIBRARY}" DIRECTORY)
  list(APPEND CMAKE_INSTALL_RPATH "${azure_storage_cpplite_LIBRARY_PATH}")
endif()

set(azure_storage_cpplite_LIBRARIES ${azure_storage_cpplite_LIBRARY} ${CURL_LIBRARIES} PkgConfig::uuid OpenSSL::SSL OpenSSL::Crypto)
set(azure_storage_cpplite_INCLUDE_DIRS ${azure_storage_cpplite_INCLUDE_DIR})
