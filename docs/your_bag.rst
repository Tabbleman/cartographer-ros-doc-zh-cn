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

========================================
运行Cartographer ROS在你自己的bag文件上
========================================

现在你已经在几个提供的bag文件上运行了Cartographer ROS，你可以继续使Cartographer在你自己的数据上工作。
找到一个你想用来进行SLAM的``.bag``记录，并按照本教程进行操作。

.. warning:: 当你想运行cartographer_ros时，可能需要通过运行``source install_isolated/setup.bash``来设置你的ROS环境（如果你的shell是zsh，请将bash替换为zsh）。

验证你的bag文件
=================

Cartographer ROS提供了一个名为``cartographer_rosbag_validate``的工具来自动分析你的bag文件中的数据。
在尝试调整Cartographer以处理错误数据之前，运行此工具通常是个好主意。

该工具利用了Cartographer作者的经验，可以检测出bag文件中常见的各种错误。
例如，如果检测到``sensor_msgs/Imu``主题，工具会确保重力向量未从IMU测量中移除，因为Cartographer使用重力标准来确定地面的方向。

该工具还可以提供一些改进数据质量的建议。
例如，对于Velodyne LIDAR，建议每个UDP包发送一个``sensor_msgs/PointCloud2``消息，而不是每次旋转发送一个消息。
通过这种粒度，Cartographer可以解扭曲由于机器人运动导致的点云变形，从而获得更好的重建效果。

如果你已经设置了Cartographer ROS环境，可以简单地运行该工具：

.. code-block:: bash

    cartographer_rosbag_validate -bag_filename your_bag.bag

创建一个.lua配置文件
===========================

Cartographer高度灵活，可以配置为在各种机器人上工作。
机器人配置是从一个必须由Lua脚本定义的``options``数据结构读取的。
示例配置定义在``src/cartographer_ros/cartographer_ros/configuration_files``中，并安装在``install_isolated/share/cartographer_ros/configuration_files/``中。

.. note:: 理想情况下，.lua配置应该是机器人特定的，而不是bag文件特定的。

你可以从复制一个示例开始，然后根据自己的需要进行调整。如果你想使用3D SLAM：

.. code-block:: bash

    cp install_isolated/share/cartographer_ros/configuration_files/backpack_3d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua

如果你想使用2D SLAM：

.. code-block:: bash

    cp install_isolated/share/cartographer_ros/configuration_files/backpack_2d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua

然后可以编辑``my_robot.lua``以满足你机器人的需要。
在``options``块中定义的值决定了Cartographer ROS前端应该如何与你的bag文件接口。
在``options``段落之后定义的值用于调整Cartographer的内部工作，我们现在将忽略这些。

.. seealso:: `Cartographer ROS配置值参考文档`_和`Cartographer配置值参考文档`_。

.. _Cartographer ROS配置值参考文档: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

.. _Cartographer配置值参考文档: https://google-cartographer.readthedocs.io/en/latest/configuration.html

在你需要调整的值中，你可能需要提供你的环境和机器人中的TF框架ID，包括``map_frame``、``tracking_frame``、``published_frame``和``odom_frame``。

.. note:: 你可以通过在bag文件中发布``/tf``主题来分发机器人的TF树，也可以在``.urdf``机器人定义中定义它。

.. warning:: 你应该信任你的姿态！机器人和IMU或LIDAR之间的链接上的小偏移可能导致地图重建不一致。Cartographer通常可以校正小的姿态误差，但不是所有的！

你需要定义的其他值与你想使用的传感器的数量和类型有关。

- ``num_laser_scans``：你将使用的``sensor_msgs/LaserScan``主题的数量。
- ``num_multi_echo_laser_scans``：你将使用的``sensor_msgs/MultiEchoLaserScan``主题的数量。
- ``num_point_clouds``：你将使用的``sensor_msgs/PointCloud2``主题的数量。

你还可以启用地标和GPS作为额外的定位源，使用``use_landmarks``和``use_nav_sat``。``options``块中的其他变量通常应该保持不变。

.. note:: 即使你使用2D SLAM，地标也是3D对象，如果仅在2D平面上查看，由于它们的第三维度，可能会误导你。

然而，有一个全局变量绝对需要根据你的bag文件的需要进行调整：``TRAJECTORY_BUILDER_3D.num_accumulated_range_data``或``TRAJECTORY_BUILDER_2D.num_accumulated_range_data``。
这个变量定义了构建完整扫描（通常是一次完整旋转）所需的消息数量。
如果你按照``cartographer_rosbag_validate``的建议，每次扫描使用100条ROS消息，你可以将此变量设置为100。
如果你有两个测距传感器（例如两个LIDAR）同时提供它们的完整扫描，则应将此变量设置为2。

为你的SLAM场景创建.launch文件
============================================

你可能已经注意到，每个演示都使用不同的roslaunch命令运行。
Cartographer的推荐用法确实是为每个机器人和SLAM类型提供一个自定义的``.launch``文件。
示例``.launch``文件定义在``src/cartographer_ros/cartographer_ros/launch``中，并安装在``install_isolated/share/cartographer_ros/launch/``中。

从复制一个提供的示例开始：

.. code-block:: bash

    cp install_isolated/share/cartographer_ros/launch/backpack_3d.launch install_isolated/share/cartographer_ros/launch/my_robot.launch
    cp install_isolated/share/cartographer_ros/launch/demo_backpack_3d.launch install_isolated/share/cartographer_ros/launch/demo_my_robot.launch
    cp install_isolated/share/cartographer_ros/launch/offline_backpack_3d.launch install_isolated/share/cartographer_ros/launch/offline_my_robot.launch
    cp install_isolated/share/cartographer_ros/launch/demo_backpack_3d_localization.launch install_isolated/share/cartographer_ros/launch/demo_my_robot_localization.launch
    cp install_isolated/share/cartographer_ros/launch/assets_writer_backpack_3d.launch install_isolated/share/cartographer_ros/launch/assets_writer_my_robot.launch

- ``my_robot.launch``用于在机器人上使用真实传感器数据在线（实时）执行SLAM。
- ``demo_my_robot.launch``用于从开发机器上运行，并需要一个``bag_filename``参数来重放记录的数据。此启动文件还会生成一个rviz窗口，用于可视化Cartographer的状态。
- ``offline_my_robot.launch``与``demo_my_robot.launch``非常相似，但尝试尽可能快地执行SLAM。这可以显著加快地图构建速度。此启动文件还可以使用提供给``bag_filenames``参数的多个bag文件。
- ``demo_my_robot_localization.launch``与``demo_my_robot.launch``非常相似，但需要一个``load_state_filename``参数，指向先前Cartographer执行的``.pbstream``记录。先前的记录将用作预先计算的地图，Cartographer只在此地图上执行定位。
- ``assets_writer_my_robot.launch``用于从先前Cartographer执行的``.pbstream``记录中提取数据。

再次提醒，需要对这些文件进行一些调整以适应你的机器人。

- 所有给``-configuration_basename``的参数应该调整为指向``my_robot.lua``。
- 如果你决定使用``.urdf``描述你的机器人，应将描述放在``install_isolated/share/cartographer_ros/urdf``中，并调整``robot_description``参数以指向你的文件名。
- 如果你决定使用``/tf``消息，可以删除``robot_description``参数、``robot_state_publisher``节点和以``-urdf``开头的行。
- 如果你的bag文件或传感器发布的主题名称与Cartographer ROS预期的不匹配，可以使用``<remap>``元素重定向你的主题。预期的主题名称取决于你使用的测距设备的类型。

.. note::

    - IMU主题预期为“imu”
    - 如果你只使用一个``sensor_msgs/LaserScan``主题，预期名称为``scan``。如果有多个，应该命名为``scan_1``、``scan_2``等。
    - 如果你只使用一个``sensor_msgs/MultiEchoLaserScan``主题，预期名称为``echoes``。如果有多个，应该命
