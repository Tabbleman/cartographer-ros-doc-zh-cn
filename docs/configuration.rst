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

=========================================
Lua 配置参考文档
=========================================

请注意，Cartographer 的 ROS 集成使用 `tf2`_，因此所有的帧 ID 都应仅包含帧名（小写并带下划线），不包含前缀或斜杠。参见 `REP 105`_ 以了解常用的坐标帧。

请注意，在 Cartographer 的 ROS 集成中，主题名称以*基本*名称给出（参见 `ROS Names`_）。这意味着 Cartographer 节点的用户需要重新映射，或者将其放入命名空间中。

以下是 Cartographer 的 ROS 集成顶级选项，所有这些选项必须在 Lua 配置文件中指定：

map_frame
  用于发布子地图的 ROS 帧 ID，姿态的父帧，通常为 "map"。

tracking_frame
  由 SLAM 算法跟踪的帧的 ROS 帧 ID。如果使用 IMU，它应该位于其位置，尽管它可能会旋转。常见的选择是 "imu_link"。

published_frame
  用于发布姿态的子帧的 ROS 帧 ID。例如，如果系统的其他部分提供了 "odom" 帧，则设置为 "odom"。在这种情况下，将发布 "odom" 在 *map_frame* 中的姿态。否则，设置为 "base_link" 可能是合适的。

odom_frame
  仅在 *provide_odom_frame* 为 true 时使用。用于发布（未闭环的）局部 SLAM 结果的 *published_frame* 和 *map_frame* 之间的帧。通常为 "odom"。

provide_odom_frame
  如果启用，本地的、未闭环的、连续的姿态将作为 *odom_frame* 在 *map_frame* 中发布。

publish_frame_projected_to_2d
  如果启用，发布的姿态将限制为纯 2D 姿态（没有滚转、俯仰或 z 偏移）。这可以防止由于姿态外推步骤导致的 2D 模式中可能出现的平面外姿态（例如，如果姿态应发布为类似 'base-footprint' 的帧）。

use_odometry
  如果启用，订阅 `nav_msgs/Odometry`_ 主题 "odom"。在这种情况下必须提供里程计信息，并且该信息将包含在 SLAM 中。

use_nav_sat
  如果启用，订阅 `sensor_msgs/NavSatFix`_ 主题 "fix"。在这种情况下必须提供导航数据，并且该信息将包含在全局 SLAM 中。

use_landmarks
  如果启用，订阅 `cartographer_ros_msgs/LandmarkList`_ 主题 "landmarks"。必须提供地标，作为 `cartographer_ros_msgs/LandmarkEntry`_ 在 `cartographer_ros_msgs/LandmarkList`_ 中的一部分。如果提供 `cartographer_ros_msgs/LandmarkEntry`_ 数据，该信息将根据 `cartographer_ros_msgs/LandmarkEntry`_ 的 ID 包含在 SLAM 中。`cartographer_ros_msgs/LandmarkList`_ 应以与其他传感器相当的采样率提供。列表可以为空，但必须提供，因为 Cartographer 严格按时间顺序对传感器数据进行排序，以使地标具有确定性。然而，可以将轨迹生成器选项 "collate_landmarks" 设置为 false，以允许一种不确定但也不阻塞的方法。

num_laser_scans
  订阅的激光扫描主题的数量。为一个激光扫描仪订阅 `sensor_msgs/LaserScan`_ 主题 "scan"，或为多个激光扫描仪订阅 "scan_1"、"scan_2" 等主题。

num_multi_echo_laser_scans
  订阅的多回波激光扫描主题的数量。为一个激光扫描仪订阅 `sensor_msgs/MultiEchoLaserScan`_ 主题 "echoes"，或为多个激光扫描仪订阅 "echoes_1"、"echoes_2" 等主题。

num_subdivisions_per_laser_scan
  将每个接收到的（多回波）激光扫描划分为点云的数量。划分扫描可以使在扫描仪移动时获取的扫描图像去扭曲。存在一个对应的轨迹生成器选项，用于将划分的扫描累积为将用于扫描匹配的点云。

num_point_clouds
  订阅的点云主题的数量。为一个测距仪订阅 `sensor_msgs/PointCloud2`_ 主题 "points2"，或为多个测距仪订阅 "points2_1"、"points2_2" 等主题。

lookup_transform_timeout_sec
  用于使用 `tf2`_ 查找转换的超时时间（以秒为单位）。

submap_publish_period_sec
  发布子地图姿态的时间间隔（以秒为单位），例如 0.3 秒。

pose_publish_period_sec
  发布姿态的时间间隔（以秒为单位），例如 5e-3 对应于 200 Hz 的频率。

publish_to_tf
  启用或禁用 TF 转换的提供。

publish_tracked_pose
  启用将跟踪的姿态作为 `geometry_msgs/PoseStamped`_ 发布到 "tracked_pose" 主题。

trajectory_publish_period_sec
  发布轨迹标记的时间间隔（以秒为单位），例如 30e-3 对应于 30 毫秒。

rangefinder_sampling_ratio
  对测距仪消息进行固定比例采样。

odometry_sampling_ratio
  对里程计消息进行固定比例采样。

fixed_frame_sampling_ratio
  对固定帧消息进行固定比例采样。

imu_sampling_ratio
  对 IMU 消息进行固定比例采样。

landmarks_sampling_ratio
  对地标消息进行固定比例采样。

.. _REP 105: http://www.ros.org/reps/rep-0105.html
.. _ROS Names: http://wiki.ros.org/Names
.. _geometry_msgs/PoseStamped: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
.. _nav_msgs/OccupancyGrid: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
.. _nav_msgs/Odometry: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
.. _sensor_msgs/LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
.. _sensor_msgs/MultiEchoLaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/MultiEchoLaserScan.html
.. _sensor_msgs/PointCloud2: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
.. _sensor_msgs/NavSatFix: http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
.. _cartographer_ros_msgs/LandmarkList: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/msg/LandmarkList.msg
.. _cartographer_ros_msgs/LandmarkEntry: https://github.com/cartographer-project/cartographer_ros/blob/4b39ee68c7a4d518bf8d01a509331e2bc1f514a0/cartographer_ros_msgs/msg/LandmarkEntry.msg
.. _tf2: http://wiki.ros.org/tf2
