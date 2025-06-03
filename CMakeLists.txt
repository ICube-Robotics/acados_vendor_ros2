cmake_minimum_required(VERSION 3.10)
project(acados_vendor_ros2)

set(ACADOS_VERSION "0.4.0")
# CMake options
option(FORCE_BUILD_VENDOR_PKG
  "Build Acados from source, even if system-installed package is available"
  ON
)
option(BUILD_ACADOS_TEMPLATE
  "Build and install the Python interface ('acados_template' package). Requires two consecutive builds to be effective!"
  ON
)

# ACADOS options on External libs
option(ACADOS_WITH_QPOASES "qpOASES solver" OFF)
option(ACADOS_WITH_DAQP "DAQP solver" OFF)
option(ACADOS_WITH_HPMPC "HPMPC solver" OFF)
option(ACADOS_WITH_QORE "QORE solver" OFF)
option(ACADOS_WITH_OOQP "OOQP solver" OFF)
option(ACADOS_WITH_QPDUNES "qpDUNES solver" OFF)
option(ACADOS_WITH_OSQP "OSQP solver" OFF)

# Setup
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # Default to C++17
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(acados ${ACADOS_VERSION} EXACT QUIET)

ament_vendor(acados_vendor_ros2
  SATISFIED ${acados_FOUND}
  VCS_URL https://github.com/acados/acados.git
  VCS_VERSION v${ACADOS_VERSION}
  CMAKE_ARGS
  "-DACADOS_WITH_QPOASES=${ACADOS_WITH_QPOASES};\
  -DACADOS_WITH_DAQP=${ACADOS_WITH_DAQP};\
  -DACADOS_WITH_HPMPC=${ACADOS_WITH_HPMPC};\
  -DACADOS_WITH_QORE=${ACADOS_WITH_QORE};\
  -DACADOS_WITH_OOQP=${ACADOS_WITH_OOQP};\
  -DACADOS_WITH_QPDUNES=${ACADOS_WITH_QPDUNES};\
  -DACADOS_WITH_OSQP=${ACADOS_WITH_OSQP}"
)
ament_export_dependencies(acados)

set(ACADOS_SOURCE_DIR ${CMAKE_INSTALL_PREFIX}/opt/acados_vendor_ros2) # lib + include
set(ACADOS_SOURCE_BUILD_DIR ${CMAKE_BINARY_DIR}/acados_vendor_ros2-prefix/src/acados_vendor_ros2) # Source files

# Source files
set(ACADOS_PYTHON_INTERFACE_PATH none) # Python package after install, set below if package built

if(${BUILD_ACADOS_TEMPLATE})
  set(ACADOS_PYTHON_INTERFACE_PCK_DIR "${ACADOS_SOURCE_BUILD_DIR}/interfaces/acados_template/acados_template")

  if(NOT EXISTS "${ACADOS_PYTHON_INTERFACE_PCK_DIR}")
    message(STATUS "Cloning Acados source")
    file(MAKE_DIRECTORY ${ACADOS_SOURCE_BUILD_DIR})
    execute_process(COMMAND git clone -c advice.detachedHead=false --quiet --recursive https://github.com/acados/acados.git -b v${ACADOS_VERSION} .
                    WORKING_DIRECTORY ${ACADOS_SOURCE_BUILD_DIR}
                    RESULT_VARIABLE ACADOS_GIT_OUT
                    OUTPUT_VARIABLE ACADOS_GIT_OUT)
  else()
    message(STATUS "${ACADOS_SOURCE_BUILD_DIR} already exists")
  endif()

  ament_python_install_package(
    acados_template
    PACKAGE_DIR ${ACADOS_PYTHON_INTERFACE_PCK_DIR}
  )
  set(ACADOS_PYTHON_INTERFACE_PATH ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/acados_template) # Python package after install
  message(
    STATUS
    "The Python interface (acados_template) \
    found in '${ACADOS_PYTHON_INTERFACE_PCK_DIR}' will be installed in \
    '${ACADOS_PYTHON_INTERFACE_PATH}'."
  )

  set(TERA_RENDERER "${ACADOS_SOURCE_DIR}/bin/t_renderer")
  if(NOT EXISTS "${TERA_RENDERER}")
    # Add empty bin directory (used to download the tera renderer)
    install(DIRECTORY DESTINATION "opt/${PROJECT_NAME}/bin")
    set(TERA_VERSION "0.0.34")
    set(TERA_URL_BASE "https://github.com/acados/tera_renderer")
    set(TERA_URL "${TERA_URL_BASE}/releases/download/v${TERA_VERSION}/t_renderer-v${TERA_VERSION}-linux")
    message(STATUS "Tera renderer not found, downloading version ${TERA_VERSION} from ${TERA_URL}")

    # Download binaries for the tera renderer
    file(DOWNLOAD
      "${TERA_URL}"
      "${TERA_RENDERER}"
      SHOW_PROGRESS
      STATUS TERA_DOWNLOAD_STATUS
    )
    list(GET TERA_DOWNLOAD_STATUS 0 TERA_DOWNLOAD_CODE)
    if(NOT TERA_DOWNLOAD_CODE EQUAL 0)
      message(FATAL_ERROR "Failed to download Tera renderer: ${TERA_DOWNLOAD_STATUS}")
    endif()

    # Make the tera renderer executable
    file(CHMOD
      ${TERA_RENDERER}
      PERMISSIONS
        OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE
    )
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  # TODO(anyone): Add unit tests ?
  # See https://github.com/ros2/libyaml_vendor/blob/rolling/CMakeLists.txt for an example
endif()

# Export 'FindAcados.cmake'
install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

# Copy additional cmake utils required by Acados
#  (to suppress findPackage warnings in downstream packages)
install(
  FILES
    #${ACADOS_SOURCE_BUILD_DIR}/cmake/FindFortranLibs.cmake
    # Note: Can triggers naming convention warnings...
    #${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/FindFORTRANLIBS.cmake
    ${ACADOS_SOURCE_BUILD_DIR}/cmake/FindOpenBLAS.cmake
  DESTINATION
    share/${PROJECT_NAME}/cmake/Modules
)

ament_package(CONFIG_EXTRAS acados_vendor_ros2-extras.cmake)
