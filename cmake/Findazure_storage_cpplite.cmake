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

set(azure_storage_cpplite_DEPENDENCIES ${CURL_LIBRARIES} PkgConfig::uuid OpenSSL::SSL OpenSSL::Crypto)

if (NOT TARGET azure_storage_cpplite::azure_storage_cpplite)
  add_library(azure_storage_cpplite::azure_storage_cpplite STATIC IMPORTED GLOBAL)
  set_target_properties(azure_storage_cpplite::azure_storage_cpplite
    PROPERTIES
      IMPORTED_LOCATION "${azure_storage_cpplite_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${azure_storage_cpplite_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "${azure_storage_cpplite_DEPENDENCIES}"
  )
endif()

set(azure_storage_cpplite_LIBRARIES ${azure_storage_cpplite_LIBRARY} ${azure_storage_cpplite_DEPENDENCIES})
set(azure_storage_cpplite_INCLUDE_DIRS ${azure_storage_cpplite_INCLUDE_DIR})
