cmake_minimum_required(VERSION 3.10)
project(acados_vendor_ros2)

# CMake options
option(FORCE_BUILD_VENDOR_PKG
  "Build Acados from source, even if system-installed package is available"
  ON
)
option(BUILD_ACADOS_TEMPLATE
  "Build and install the Python interface ('acados_template' package). Requires two consecutive builds to be effective!"
  ON
)

# Setup
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # Default to C++17
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(acados 0.2.5 EXACT QUIET)

ament_vendor(acados_vendor_ros2
  SATISFIED ${acados_FOUND}
  VCS_URL https://github.com/acados/acados.git
  VCS_VERSION v0.2.5
)
ament_export_dependencies(acados)

set(ACADOS_SOURCE_DIR ${CMAKE_INSTALL_PREFIX}/opt/acados_vendor_ros2) # lib + include
set(ACADOS_SOURCE_BUILD_DIR ${CMAKE_BINARY_DIR}/acados_vendor_ros2-prefix/src/acados_vendor_ros2) # Source files
set(ACADOS_PYTHON_INTERFACE_PATH none) # Python package after install, set below if package built

if(${BUILD_ACADOS_TEMPLATE})
  set(ACADOS_PYTHON_INTERFACE_PCK_DIR "${ACADOS_SOURCE_BUILD_DIR}/interfaces/acados_template/acados_template")
  if(EXISTS "${ACADOS_PYTHON_INTERFACE_PCK_DIR}/__init__.py")
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
  else()
    # Force a CMAKE regeneration at next 'colcon build' call
    include("${CMAKE_CURRENT_LIST_DIR}/cmake/force_regenerate.cmake")
    force_regenerate_at_next_build()

    message(
      WARNING
      "The Python interface (acados_template) could not be built \
      because the external project was not yet downloaded!\n \
      Please re-build the project once again.\n \
      Note: if it doesn't work, please try with the cmake arg '-DBUILD_ACADOS_TEMPLATE=ON'!"
    )
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
    # Exclude auto generated c-code and CMakeLists
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/FindFortranLibs.cmake
  )

  ament_lint_auto_find_test_dependencies()

  # TODO(tpoignonec): Extra unit tests ?
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

# Add empty bin directory (used to download the tera renderer)
install(DIRECTORY DESTINATION "opt/${PROJECT_NAME}/bin")

ament_package(CONFIG_EXTRAS acados_vendor_ros2-extras.cmake)
