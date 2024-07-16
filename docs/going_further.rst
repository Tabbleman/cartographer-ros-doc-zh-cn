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

=============
更进一步
=============

Cartographer不仅是一个优秀的SLAM算法，它还附带了一个功能齐全的实现，提供了许多“额外”功能。
本页列出了一些鲜为人知的功能。

更多输入
==========

如果你有一个发布在 ``nav_msgs/Odometry``主题上的里程计（例如轮式编码器）并希望使用它来提高Cartographer的定位，可以在你的``.lua``配置文件中添加输入：

..  code-block:: lua

    use_odometry = true

消息将会在 ``odom``主题上被接收。

发布在名为 ``fix``的 ``sensor_msgs/NavSatFix``主题上的GPS可以提高全局SLAM的效果：

..  code-block:: lua

    use_nav_sat = true

对于发布在名为 ``landmark``的 ``cartographer_ros_msgs/LandmarkList``（`在cartographer_ros中定义的消息`_）主题上的地标：

..  code-block:: lua

    use_landmarks = true

.. _在cartographer_ros中定义的消息: https://github.com/cartographer-project/cartographer_ros/blob/4b39ee68c7a4d518bf8d01a509331e2bc1f514a0/cartographer_ros_msgs/msg/LandmarkList.msg

仅定位模式
=================

如果你有一个满意的地图并希望减少计算量，可以使用Cartographer的仅定位模式，该模式将在现有地图上运行SLAM而不构建新地图。
通过在运行 ``cartographer_node``时使用 ``-load_state_filename``参数，并在你的lua配置中定义以下行来启用此模式：

..  code-block:: lua

    TRAJECTORY_BUILDER.pure_localization_trimmer = {
        max_submaps_to_keep = 3,
    }

IMU校准
===============

在进行全局优化时，Ceres会尝试改进IMU与测距传感器之间的姿态。
通过大量回环约束（例如机器人沿直线前进然后返回）进行的精心采集可以提高这些校正的质量，并成为可靠的姿态校正源。
然后你可以将Cartographer作为校准过程的一部分，以提高机器人外参校准的质量。

多轨迹SLAM
=======================

Cartographer可以从多个并行发出数据的机器人执行SLAM。
全局SLAM能够检测共享路径，并在可能时合并由不同机器人构建的地图。
这是通过使用两个ROS服务 ``start_trajectory``和 ``finish_trajectory``实现的。（有关其使用的详细信息，请参阅ROS API参考文档）

使用gRPC进行云集成
===========================

Cartographer围绕Protobuf消息构建，这使其非常灵活和互操作。
这种架构的一个优势是很容易在互联网上分布式运行。
典型的用例是一个在已知地图上导航的机器人队列，他们可以在远程强大的集中定位服务器上运行SLAM算法，该服务器运行一个多轨迹的Cartographer实例。

**待办**: 有关如何开始使用gRPC Cartographer实例的说明
