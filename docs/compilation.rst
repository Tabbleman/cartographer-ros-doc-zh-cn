.. Copyright 2018 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

==========================
在oe2403上编译 Cartographer ROS
==========================

系统要求
===================
当前默认你已经在x86_64的os2403完成安装了ros2 相关的包。
Cartographer ROS 的要求与 `Cartographer 的要求`_ 相同。

当前支持以下 `ROS 发行版`_：

* Humble

.. _Cartographer 的要求: https://google-cartographer.readthedocs.io/en/latest/#system-requirements
.. _ROS 发行版: http://wiki.ros.org/Distributions

构建与安装
=======================

为了在openEuler上构建 Cartographer ROS，我们推荐使用 `colcon <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_ 。为了更快的构建，我们还推荐使用 `Ninja <https://ninja-build.org>`_。

在使用 ROS Humble 的 openEuler 上，使用以下命令安装上述工具：

.. code-block:: bash

    sudo dnf update
    sudo dnf install python3-colcon-common-extensions python3-rosdep ninja-build

安装工具后，在 'ros2_ws' 中创建一个新的 cartographer_ros 工作区。

.. code-block:: bash

    mkdir -p ros2_ws/src
    cd ros2_ws/src
    git clone https://gitee.com/src-openeuler/cartographer_ros.git
    cd cartographer_ros
    git checkout humble 
    tar xf ros-humble-cartographer-ros_2.0.9000.orig.tar.gz
    tar xf ros-humble-cartographer-ros-msgs_2.0.9000.orig.tar.gz
    tar xf ros-humble-cartographer-rviz_2.0.9000.orig.tar.gz

现在你需要给 cartographer_ros 打上补丁。

.. code-block:: bash
    cd ros-humble-cartographer-ros-2.0.9000/
    patch -p1 < ../cartographer-ros-adapt-glog-0.6.0.patch 
    patch -p1 < ../cartographer-ros-fix-multiple-definition-error.patch 
    patch -p1 < ../cartographer-ros-fix-absl.patch 
    patch -p1 < ../cartographer-ros-fix-link.patch 

还有rviz的补丁

.. code-block:: bash
    cd ../ros-humble-cartographer-rviz-2.0.9000
    patch -p1 < ../cartographer-rviz-fix-absl.patch 

构建并安装。

.. code-block:: bash
    cd ../../       # 现在你应该在 ros2_ws/下面
    colcon build    # 这将花去非常多的时间，don't panic.

.. _abseil-cpp: https://abseil.io/
