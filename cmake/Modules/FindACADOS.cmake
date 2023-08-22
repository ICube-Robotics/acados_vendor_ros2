# Copyright 2023 ICUBE Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Thibault Poignonec (tpoignonec@unistra.fr)

include(FindPackageHandleStandardArgs)
find_package(acados CONFIG QUIET)
if(acados_FOUND)
  find_package_handle_standard_args(acados FOUND_VAR acados_FOUND CONFIG_MODE)
else()
  # Otherwise, rely on pkg-config
  find_package(PkgConfig QUIET)

  if(PKG_CONFIG_FOUND)
    pkg_check_modules(ACADOS_PKG_CONFIG IMPORTED_TARGET acados)
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