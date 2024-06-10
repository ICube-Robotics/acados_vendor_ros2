# acados_vendor_ros2
Simple ros2 vendor for Acados.

![version](https://img.shields.io/badge/version-0.3.2-blue)
[![CI (humble)](https://github.com/ICube-Robotics/acados_vendor_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/ICube-Robotics/acados_vendor_ros2/actions/workflows/ci.yml)
[![Build tests (jazzy)](../../actions/workflows/ci-jazzy.yaml/badge.svg?branch=main)](../../actions/workflows/ci-jazzy.yaml?query=branch:main)
[![Build tests (rolling)](../../actions/workflows/ci-rolling.yaml/badge.svg?branch=main)](../../actions/workflows/ci-rolling.yaml?query=branch:main)


## Installation

```bash
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/ICube-Robotics/acados_vendor_ros2.git

# Download the source and install the c interface (lib "acados")
cd ..
rosdep install --ignore-src --from-paths . -y -r
colcon build && colcon build
```
Note that `colcon build` is called twice.

> N.B., this is necessary because `ament_vendor(...)` takes effect after the `ament_python_install_package(...)` command.
However, the later tests for the existence of an `__init__.py` file in the path, hence resulting in an error since the Python package is not yet present.

>**TODO:** Allow unique build to avoid potential downstream build tests failures.

>**NOTE:** If the `ament_cmake_vendor_package` is missing, you can install it manually:
```bash
sudo apt install ros-<ROS_DISTRO>-ament-cmake-vendor-package
```

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

> `XXX.py` :
```python
from acados_template import AcadosModel, AcadosOcp
...
model = AcadosModel()
...
```
