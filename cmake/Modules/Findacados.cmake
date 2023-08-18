include(FindPackageHandleStandardArgs)
find_package(acados CONFIG QUIET)
if(acados_FOUND)
  find_package_handle_standard_args(acados FOUND_VAR acados_FOUND CONFIG_MODE)
else()
  # Otherwise, rely on pkg-config
  find_package(PkgConfig QUIET)

  if(PKG_CONFIG_FOUND)
    pkg_check_modules(ACADOS_PKG_CONFIG IMPORTED_TARGET acados>=2.2.0)
    find_package_handle_standard_args(acados DEFAULT_MSG ACADOS_PKG_CONFIG_FOUND)

    if(acados_FOUND)
      if(NOT TARGET acados)
        add_library(acados INTERFACE IMPORTED)
        set_property(TARGET acados PROPERTY INTERFACE_LINK_LIBRARIES PkgConfig::ACADOS_PKG_CONFIG)
      endif()
      set(acados_LIBRARIES acados)
      set(acados_VERSION ${ACADOS_PKG_CONFIG_VERSION})
    endif()
  endif()
endif()