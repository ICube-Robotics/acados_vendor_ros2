ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/src/acados_vendor_ros2
RUN apt update && apt upgrade -y
RUN apt install -y python3-pip
RUN apt install -y git tree clang-format-14 cppcheck
RUN pip install pre-commit
