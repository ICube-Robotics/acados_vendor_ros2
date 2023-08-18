cmake_minimum_required(VERSION 3.10)
project(acados_vendor_ros2)

#option(FORCE_BUILD_VENDOR_PKG
#  "Build Acados from source, even if system-installed package is available"
#  OFF)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(acados 0.2.2 EXACT QUIET)

ament_vendor(acados_vendor_ros2
  SATISFIED ${acados_FOUND}
  VCS_URL https://github.com/acados/acados.git
  #VCS_VERSION 0.2.2
  #CMAKE_ARGS
  #  "-DCMAKE_C_FLAGS:"
)
ament_export_dependencies(acados)
if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_lint_cmake()
  ament_copyright()
  ament_xmllint()
  
  #[===[
  if(TARGET acados_vendor_ros2)
    add_library(acados_library INTERFACE)
    externalproject_get_property(acados_vendor_ros2 INSTALL_DIR)
    target_include_directories(acados_library INTERFACE 
      "${INSTALL_DIR}/include"
      "${INSTALL_DIR}/blasfeo/include"
      "${INSTALL_DIR}/hpipm/include"
    )
    target_link_directories(acados_library INTERFACE "${INSTALL_DIR}/lib")
    target_link_libraries(acados_library INTERFACE ${CMAKE_LINK_LIBRARY_FLAG}acados)
    add_dependencies(acados_library acados_vendor_ros2)
  else()
    add_library(acados_library ALIAS acados_vendor_ros2)
  endif()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rcpputils REQUIRED)
  
  ament_add_gtest(test_c_interface test/test_acados_c_interface.cpp
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

  if(TARGET test_c_interface)
    target_link_libraries(test_c_interface acados_library)
    ament_target_dependencies(test_c_interface rcpputils)
  endif()
  ]===]
  
  # TODO: Extra unit tests ?
  # See https://github.com/ros2/libyaml_vendor/blob/rolling/CMakeLists.txt for an example
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS acados_vendor_ros2-extras.cmake)