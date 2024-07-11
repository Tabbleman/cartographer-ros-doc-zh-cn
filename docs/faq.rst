.. Copyright 2016 The Cartographer Authors

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
常见问题解答
==========================

为什么3D包中的激光数据率高于VLP-16报告的最大20 Hz旋转速度？
------------------------------------------------------------

示例包中的VLP-16配置为以20 Hz旋转。然而，VLP-16发送的UDP包的频率要高得多，并且与旋转频率无关。示例包包含每个UDP包的sensor_msgs/PointCloud2__，而不是每次旋转一个。

__ http://www.ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html

在相应的Cartographer配置文件__中，您可以看到TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160，这意味着我们将160个每个UDP包的点云累积为一个更大的点云，该点云通过结合恒定速度和IMU测量进行运动估计，用于匹配。由于有两个VLP-16，160个UDP包足以进行大约2次旋转，每个VLP-16一次。

__ https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/configuration_files/backpack_3d.lua

为什么3D SLAM需要IMU数据而2D不需要？
--------------------------------------

在2D中，Cartographer支持运行相关扫描匹配器，这通常用于查找闭环约束，用于局部SLAM。它计算成本高，但通常可以使引入里程计或IMU数据变得不必要。2D还具有假设平坦世界的好处，即上方向是隐式定义的。

在3D中，IMU主要用于测量重力。重力是一个有吸引力的测量量，因为它不会漂移，并且是一个非常强的信号，通常包含大部分测量的加速度。需要重力有两个原因：

1. 在3D中，对世界没有任何假设。为了正确对齐生成的轨迹和地图，使用重力来定义z方向。

2. 一旦确定了重力方向，可以很好地从IMU读数中导出滚转和俯仰。这通过减少这些维度中的搜索窗口为扫描匹配器节省了工作。

如何在没有rviz支持的情况下构建cartographer_ros？
------------------------------------------

最简单的解决方案是在cartographer_rviz包目录中创建一个名为CATKIN_IGNORE__的空文件。

__ http://wiki.ros.org/catkin/workspaces

如何修复“您调用了InitGoogleLogging()两次！”错误？
-------------------------------------------------

用glog后端构建rosconsole可能会导致此错误。使用log4cxx或打印后端，可以通过ROSCONSOLE_BACKEND CMake参数选择，来避免此问题。
