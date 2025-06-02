# acados_vendor_ros2
Simple ros2 vendor for Acados.

![version](https://img.shields.io/badge/version-0.3.6-blue)
[![CI (humble)](https://github.com/ICube-Robotics/acados_vendor_ros2/actions/workflows/ci-humble.yml/badge.svg)](https://github.com/ICube-Robotics/acados_vendor_ros2/actions/workflows/ci-humble.yml)
[![Build tests (jazzy)](../../actions/workflows/ci-jazzy.yaml/badge.svg?branch=main)](../../actions/workflows/ci-jazzy.yaml?query=branch:main)
[![Build tests (rolling)](../../actions/workflows/ci-rolling.yaml/badge.svg?branch=main)](../../actions/workflows/ci-rolling.yaml?query=branch:main)

## Installation (humble)

```bash
git clone https://github.com/ICube-Robotics/acados_vendor_ros2.git
cd acados_vendor_ros2

rosdep install --from-paths . -y --ignore-src
colcon build
```

## Installation (jazzy and later)

```bash
git clone https://github.com/ICube-Robotics/acados_vendor_ros2.git
cd acados_vendor_ros2

PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths . -y --ignore-src
colcon build
```
> **Tip:** Since Ubuntu 24.04, pip restricts installing Python packages to system locations by default. The `PIP_BREAK_SYSTEM_PACKAGES=1` flag allows `rosdep` to install Python dependencies even when system Python is used.
> You can avoid using the flag if you manually install the `casadi` binaries beforehand as a system package or if you use a virtual environment (**later option recommended**).

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
