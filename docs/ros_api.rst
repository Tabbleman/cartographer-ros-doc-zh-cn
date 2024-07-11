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

===============================
ROS API参考文档
===============================

.. image:: nodes_graph_demo_2d.jpg

Cartographer节点
=================

`cartographer_node`_ 是用于在线实时SLAM的SLAM节点。

.. _cartographer_node: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/node_main.cc

命令行标志
------------------

使用 ``--help`` 标志调用节点以查看所有可用选项。

订阅的主题
-----------------

以下距离数据主题是互斥的。至少需要一个距离数据源。

scan (`sensor_msgs/LaserScan`_)
  支持2D和3D（例如使用轴向旋转的平面激光扫描仪）。
  如果在 :doc:`configuration` 中将 *num_laser_scans* 设置为1，则此主题将用作SLAM的输入。如果 *num_laser_scans* 大于1，则多个编号的扫描主题（即scan_1，scan_2，scan_3，... 直到 *num_laser_scans*）将用作SLAM的输入。

echoes (`sensor_msgs/MultiEchoLaserScan`_)
  支持2D和3D（例如使用轴向旋转的平面激光扫描仪）。
  如果在 :doc:`configuration` 中将 *num_multi_echo_laser_scans* 设置为1，则此主题将用作SLAM的输入。仅使用第一个回波。如果 *num_multi_echo_laser_scans* 大于1，则多个编号的回波主题（即echoes_1，echoes_2，echoes_3，... 直到 *num_multi_echo_laser_scans*）将用作SLAM的输入。

points2 (`sensor_msgs/PointCloud2`_)
  如果在 :doc:`configuration` 中将 *num_point_clouds* 设置为1，则此主题将用作SLAM的输入。如果 *num_point_clouds* 大于1，则多个编号的points2主题（即points2_1，points2_2，points2_3，... 直到 *num_point_clouds*）将用作SLAM的输入。

以下附加传感器数据主题也可以提供：

imu (`sensor_msgs/Imu`_)
  支持2D（可选）和3D（必需）。此主题将用作SLAM的输入。

odom (`nav_msgs/Odometry`_)
  支持2D（可选）和3D（可选）。如果在 :doc:`configuration` 中启用 *use_odometry*，则此主题将用作SLAM的输入。

.. TODO: 添加NavSatFix？地标？

发布的主题
----------------

scan_matched_points2 (`sensor_msgs/PointCloud2`_)
  用于扫描到子地图匹配的点云。此点云可能会根据 :doc:`configuration` 进行过滤和投影。

submap_list (`cartographer_ros_msgs/SubmapList`_)
  所有子地图的列表，包括每个子地图的姿态和最新版本号，跨越所有轨迹。

tracked_pose (`geometry_msgs/PoseStamped`_)
  仅在参数 ``publish_tracked_pose`` 设置为 ``true`` 时发布。跟踪框架相对于地图框架的姿态。

服务
--------

所有服务响应还包括一个 ``StatusResponse``，其中包含一个 ``code`` 和一个 ``message`` 字段。
为了保持一致性，整数 ``code`` 等同于 `gRPC`_ API 中使用的状态代码。

.. _gRPC: https://developers.google.com/maps-booking/reference/grpc-api/status_codes

submap_query (`cartographer_ros_msgs/SubmapQuery`_)
  获取请求的子地图。

start_trajectory (`cartographer_ros_msgs/StartTrajectory`_)
  使用默认传感器主题和提供的配置启动轨迹。可以选择指定初始姿态。返回分配的轨迹ID。

trajectory_query (`cartographer_ros_msgs/TrajectoryQuery`_)
  返回来自姿态图的轨迹数据。

finish_trajectory (`cartographer_ros_msgs/FinishTrajectory`_)
  通过运行最终优化来完成给定 `trajectory_id` 的轨迹。

write_state (`cartographer_ros_msgs/WriteState`_)
  将当前内部状态写入磁盘到 `filename` 中。文件通常会出现在 `~/.ros` 或设置的 `ROS_HOME` 中。此文件可以用作 `assets_writer_main` 的输入，以生成概率网格、X射线或PLY文件。

get_trajectory_states (`cartographer_ros_msgs/GetTrajectoryStates`_)
  返回轨迹的ID和状态。
  例如，这可以用于从单独的节点观察Cartographer的状态。

read_metrics (`cartographer_ros_msgs/ReadMetrics`_)
  返回Cartographer所有内部指标的最新值。
  运行时指标的收集是可选的，必须在节点中使用 ``--collect_metrics`` 命令行标志激活。

必需的tf变换
----------------------

.. image:: frames_demo_2d.jpg

必须提供从所有传入传感器数据帧到 :doc:`configured
<configuration>` *tracking_frame* 和 *published_frame* 的变换。
通常，这些由 `robot_state_publisher` 或 `static_transform_publisher` 周期性发布。

提供的tf变换
----------------------

提供 :doc:`configured <configuration>` *map_frame* 和 *published_frame* 之间的变换，除非参数 ``publish_to_tf`` 设置为 ``false``。

如果在 :doc:`configuration` 中启用了 *provide_odom_frame*，还将提供 :doc:`configured <configuration>` *odom_frame* 和 *published_frame* 之间的连续（即不受闭环影响）变换。

.. _robot_state_publisher: http://wiki.ros.org/robot_state_publisher
.. _static_transform_publisher: http://wiki.ros.org/tf#static_transform_publisher
.. _cartographer_ros_msgs/FinishTrajectory: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/FinishTrajectory.srv
.. _cartographer_ros_msgs/SubmapList: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/msg/SubmapList.msg
.. _cartographer_ros_msgs/SubmapQuery: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/SubmapQuery.srv
.. _cartographer_ros_msgs/StartTrajectory: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/StartTrajectory.srv
.. _cartographer_ros_msgs/TrajectoryQuery: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/TrajectoryQuery.srv
.. _cartographer_ros_msgs/WriteState: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/WriteState.srv
.. _cartographer_ros_msgs/GetTrajectoryStates: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/GetTrajectoryStates.srv
.. _cartographer_ros_msgs/ReadMetrics: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/ReadMetrics.srv
.. _geometry_msgs/PoseStamped: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
.. _nav_msgs/OccupancyGrid: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
.. _nav_msgs/Odometry: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
.. _sensor_msgs/Imu: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
.. _sensor_msgs/LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
.. _sensor_msgs/MultiEchoLaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/MultiEchoLaserScan.html
.. _sensor_msgs/PointCloud2: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html

离线节点
============

`offline_node`_ 是处理一袋传感器数据的最快SLAM方式。
它不监听任何主题，而是从命令行提供的一组bags中读取TF和传感器数据。
它还发布了一个随着传感器数据推进的时钟，即取代 ``rosbag play``。
在所有其他方面，它的行为与 ``cartographer_node`` 相同。
每个bag将在最终状态中成为一个单独的轨迹。
一旦处理完所有数据，它将写出最终的Cartographer状态并退出。

.. _offline_node: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/offline_node_main.cc


发布的主题
----------------

除了在线节点发布的主题之外，此节点还发布：

~bagfile_progress (`cartographer_ros_msgs/BagfileProgress`_)
  包文件处理进度，包括有关当前处理的包的详细信息，这些信息将以可通过 ``~bagfile_progress_pub_interval`` ROS参数指定的预定义间隔发布。

.. _cartographer_ros_msgs/BagfileProgress: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/msg/BagfileProgress.msg

参数
----------

~bagfile_progress_pub_interval (double, default=10.0):
  以秒为单位发布包文件处理进度的间隔。

占用栅格节点
===================

`occupancy_grid_node`_ 监听由SLAM发布的子地图，从中构建一个ROS占用栅格并发布。
此工具对于需要单一整体地图的旧节点是有用的，直到新的导航堆栈可以直接处理Cartographer的子地图。
生成地图是昂贵且缓慢的，因此地图更新的顺序是以秒为单位。
您可以使用命令行选项选择性地包含/排除来自冻结（静态）或活动轨迹的子地图。
使用 ``--help`` 标志调用节点以查看这些选项。

.. _occupancy_grid_node: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc

订阅的主题
-----------------

它仅订阅Cartographer的 ``submap_list`` 主题。

发布的主题
----------------

map (`nav_msgs/OccupancyGrid`_)
  如果订阅，节点将连续计算并发布地图。
  更新之间的时间将随着地图大小的增加而增加。对于更快的更新，请使用子地图API。

Pbstream地图发布节点
===========================

`pbstream_map_publisher`_ 是一个简单的节点，它从序列化的Cartographer状态（pbstream格式）创建一个静态占用栅格。
如果实时更新不重要，这是occupancy grid node的高效替代方案。

.. _pbstream_map_publisher: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/pbstream_map_publisher_main.cc

订阅的主题
-----------------

无。

发布的主题
----------------

map (`nav_msgs/OccupancyGrid`_)
  发布的占用栅格主题是锁存的。
