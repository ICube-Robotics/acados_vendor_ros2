# acados_vendor_ros2
Simple ros2 vendor for Acados (v0.2.2).

[![CI](https://github.com/tpoignonec/acados_vendor_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/tpoignonec/acados_vendor_ros2/actions/workflows/ci.yml)

Author: Thibault Poignonec (tpoignonec@unistra.fr)

## Installation

```bash
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/tpoignonec/acados_vendor_ros2.git

# Download the source and install the c interface (lib "acados")
cd ..
colcon build && colcon build
```
Note that `colcon build` is called twice.

> N.B., this is necessary because `ament_vendor(...)` takes effect after the `ament_python_install_package(...)` command.
However, the later tests for the existence of an `__init__.py` file in the path, hence resulting in an error since the Python package is not yet present.

>**TODO:** Allow unique build to avoid potential downstream build tests failures.

## Usage

### For a C++ project
> `package.xml` :
```xml
...
<depend>acados_vendor_ros2</depend>
...
```

> `CMakeLists.txt` :
```cmake
...
find_package(acados_vendor_ros2)
...
ament_target_dependencies(<the_target> PUBLIC acados_vendor_ros2)
...
```
> `xxx.cpp` :
```cpp
#include "acados_c/ocp_nlp_interface.h"
...
ocp_nlp_plan_t* plan_ptr = ocp_nlp_plan_create(N);
...
```

### For a Python project

> `package.xml` :
```xml
...
<depend>acados_vendor_ros2</depend>
...
```

> `setup.py` :
Nothing special.
