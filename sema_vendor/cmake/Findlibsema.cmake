# - Find sema
# Find ADLINK SEMA libraries and headers
#
#  libsema_INCLUDE_DIRS - where to find header files, etc.
#  libsema_FOUND        - True if sema found.

include(FindPackageHandleStandardArgs)

set(sema_bsp_path $ENV{SEMA_BSP_PATH})

if (NOT sema_bsp_path)
  # Can not find SEMA_BSP_PATH, so set to /opt/sema
  set(sema_bsp_path /opt/sema)
endif()

# TODO(cwyark) Should identify what we arch we are in.
set(HOST_ARCH linux64)

set(libsema_INCLUDE_DIR ${sema_bsp_path}/include)
set(libsema_LIBRARY_DIR ${sema_bsp_path}/binary/${HOST_ARCH}/lib)

if (libsema_INCLUDE_DIR AND libsema_LIBRARY_DIR)

  set(libsema_INCLUDE_DIRS ${libsema_INCLUDE_DIR})

  add_library(semaeapi SHARED IMPORTED)
  set_target_properties(semaeapi PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${libsema_INCLUDE_DIRS}>
    IMPORTED_LOCATION ${libsema_LIBRARY_DIR}/libsemaeapi.so.3.6
    )

  add_library(sema SHARED IMPORTED)
  set_target_properties(sema PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${libsema_INCLUDE_DIRS}>
    IMPORTED_LOCATION ${libsema_LIBRARY_DIR}/libsema.so.3.6
    )

  add_library(sema_hwlinux SHARED IMPORTED)
  set_target_properties(sema_hwlinux PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${libsema_INCLUDE_DIRS}>
    IMPORTED_LOCATION ${libsema_LIBRARY_DIR}/libsema_hwlinux.so.1.7
    )

  add_library(EApi_1 SHARED IMPORTED)
  set_target_properties(EApi_1 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${libsema_INCLUDE_DIRS}>
    IMPORTED_LOCATION ${libsema_LIBRARY_DIR}/libEApi_1.so.3.6
    )
  # NOTE: SEMA 3.6 R.0 doesn't have log4cpp
  #add_library(log4cpp SHARED IMPORTED)
  #set_target_properties(log4cpp PROPERTIES
  #  INTERFACE_INCLUDE_DIRECTORIES
  #  $<BUILD_INTERFACE:${libsema_INCLUDE_DIRS}>
  #  IMPORTED_LOCATION ${libsema_LIBRARY_DIR}/liblog4cpp.so.5
  #  )

else ()
  set(libsema_INCLUDE_DIR "")
  set(libsema_INCLUDE_DIRS "")
endif ()

find_package_handle_standard_args(libsema DEFAULT_MSG
  libsema_INCLUDE_DIRS)

mark_as_advanced(
  libsema_INCLUDE_DIRS
  )
