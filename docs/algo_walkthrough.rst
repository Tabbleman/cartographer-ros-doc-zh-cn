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

.. cartographer SHA: 30f7de1a325d6604c780f2f74d9a345ec369d12d
.. cartographer_ros SHA: 44459e18102305745c56f92549b87d8e91f434fe

调优算法漫游
=============

Cartographer 是一个复杂的系统，调优它需要对其内部工作原理有很好的理解。
本页尝试直观地概述 Cartographer 使用的不同子系统及其配置值。
如果你对 Cartographer 感兴趣，不仅限于入门，你应该参考 Cartographer 论文。
它只描述了2D SLAM，但严格定义了这里描述的大部分概念。
这些概念通常也适用于3D。

W. Hess, D. Kohler, H. Rapp, 和 D. Andor,
《2D LIDAR SLAM 的实时闭环》_，发表于
*机器人与自动化（ICRA），2016 IEEE 国际会议*。
IEEE，2016。第1271–1278页。

.. _2D LIDAR SLAM 的实时闭环: https://research.google.com/pubs/pub45466.html

概述
----

.. image:: https://raw.githubusercontent.com/cartographer-project/cartographer/master/docs/source/high_level_system_overview.png
     :target: https://github.com/cartographer-project/cartographer/blob/master/docs/source/high_level_system_overview.png

Cartographer 可以看作是两个独立但相关的子系统。
第一个是 **本地 SLAM**（有时也称为 **前端** 或本地轨迹生成器）。
它的工作是构建一系列 **子地图**。
每个子地图旨在局部一致，但我们接受本地 SLAM 随时间漂移。
大多数本地 SLAM 选项可以在 `install_isolated/share/cartographer/configuration_files/trajectory_builder_2d.lua`_ 中找到用于2D，`install_isolated/share/cartographer/configuration_files/trajectory_builder_3d.lua`_ 中找到用于3D。 （在本页的其余部分，我们将 `TRAJECTORY_BUILDER_nD` 用于通用选项）

.. _install_isolated/share/cartographer/configuration_files/trajectory_builder_2d.lua: https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/trajectory_builder_2d.lua
.. _install_isolated/share/cartographer/configuration_files/trajectory_builder_3d.lua: https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/trajectory_builder_3d.lua

另一个子系统是 **全局 SLAM**（有时称为 **后端**）。
它在后台线程中运行，主要任务是找到 **闭环约束**。
它通过扫描匹配 **扫描**（在 **节点** 中收集）与子地图来实现这一点。
它还结合其他传感器数据以获得更高层次的视图，并识别最一致的全局解决方案。
在3D中，它还尝试找到重力方向。
大多数选项可以在 `install_isolated/share/cartographer/configuration_files/pose_graph.lua`_ 中找到

.. _install_isolated/share/cartographer/configuration_files/pose_graph.lua: https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/pose_graph.lua

从更高的抽象层次来看，本地 SLAM 的工作是生成好的子地图，而全局 SLAM 的工作是将它们最一致地联系在一起。

输入
----

测距传感器（例如：LIDAR）提供多个方向的深度信息。
然而，某些测量值对于 SLAM 来说是不相关的。
如果传感器部分被灰尘覆盖或指向机器人的某个部位，则某些测量距离可以被视为 SLAM 的噪声。
另一方面，某些最远的测量值也可能来自不希望的来源（反射、传感器噪声）且对 SLAM 也不相关。
为了解决这些问题，Cartographer 通过应用带通滤波器来开始，只保留在某个最小和最大范围之间的测距值。
这些最小和最大值应根据你的机器人和传感器的规格选择。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.min_range
    TRAJECTORY_BUILDER_nD.max_range

.. note::

    在2D中，Cartographer 将超过 max_range 的范围替换为 ``TRAJECTORY_BUILDER_2D.missing_data_ray_length``。 它还提供 ``max_z`` 和 ``min_z`` 值以将3D点云过滤为2D切片。

.. note::

    在 Cartographer 配置文件中，每个距离都以米为单位

测距是在机器人实际移动的某个时间段内测量的。
然而，传感器通过大 ROS 消息“批量”传递距离。
Cartographer 可以独立地考虑每个消息的时间戳，以考虑机器人运动引起的变形。
Cartographer 获取测量值的频率越高，解卷积测量值以组装可能瞬间捕获的单个连贯扫描的效果就越好。
因此，强烈建议为每个扫描（可以与另一个扫描匹配的一组测距数据）提供尽可能多的测距数据（ROS 消息）。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.num_accumulated_range_data

测距数据通常从机器人上的单个点测量，但在多个角度。
这意味着靠近的表面（例如道路）经常被击中并提供大量点。
相反，远处的物体较少被击中并提供较少的点。
为了减少点处理的计算量，我们通常需要对点云进行子采样。
然而，简单的随机采样会从我们已经有低密度测量的区域中移除点，而高密度区域仍会有更多的点。
为了解决这个密度问题，我们可以使用体素滤波器将原始点采样到固定大小的立方体中，并仅保留每个立方体的质心。

小立方体大小将导致更密集的数据表示，导致更多的计算。
大立方体大小将导致数据丢失但会更快。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.voxel_filter_size

在应用固定大小的体素滤波器后，Cartographer 还应用 **自适应体素滤波器**。
该滤波器尝试确定最佳体素大小（在最大长度内）以实现目标点数。
在3D中，使用两个自适应体素滤波器生成高分辨率和低分辨率点云，其使用将在 :ref:`local-slam` 中进行说明。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.max_length
    TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.min_num_points

惯性测量单元可以是 SLAM 的有用信息来源，因为它提供了重力（因此地面）的精确方向，并且是一个噪声但良好的整体旋转指示。
为了过滤 IMU 噪声，重力会在一定时间内被观察。
如果你使用2D SLAM，测距数据可以在没有额外信息源的情况下实时处理，因此你可以选择是否希望 Cartographer 使用 IMU。
使用3D SLAM，你需要提供 IMU，因为它被用作扫描方向的初始猜测，大大减少了扫描匹配的复杂性。

.. code-block:: lua

    TRAJECTORY_BUILDER_2D.use_imu_data
    TRAJECTORY_BUILDER_nD.imu_gravity_time_constant

.. note::

    在 Cartographer 配置文件中，每个时间值都以秒为单位

.. _local-slam:

本地 SLAM
--------

一旦扫描从多个测距数据中组装并过滤，它就准备好用于本地 SLAM 算法。
本地 SLAM 通过使用 **位姿外推器** 的初始猜测 **扫描匹配** 将新扫描插入其当前子地图构建中。
位姿外推器的想法是使用除测距仪以外的其他传感器数据来预测下一次扫描应插入子地图的位置。

有两种扫描匹配策略可用：

- ``CeresScanMatcher`` 将初始猜测作为先验并找到扫描匹配子地图的最佳位置。
  它通过插值子地图和子像素对齐扫描来实现这一点。
  这很快，但无法修复比子地图分辨率大得多的错误。
  如果你的传感器提供足够快的数据，并且你预期机器人的小速度，这将是一个不错的选择。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.ceres_scan_matcher

- ``RealTimeCorrelativeScanMatcher`` 通过暴力搜索靠近预测的扫描位置。
  它尝试在大量位姿上插入扫描并找到最一致的解决方案。
  这更慢但更鲁棒。
  对于可能较快移动的机器人和速度较慢的传感器，我们建议使用此选项。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher

.. note::

    我们发现对于具有非常精确预测的系统，可能只需要使用 CeresScanMatcher，并将 RealTimeCorrelativeScanMatcher 的权重设置为 0。
    然而，当传感器更新速率相对较低时，可能需要结合两者的优势来实现最佳性能。

由于新的测距和插入的扫描，Cartographer 会间隔重建其子地图。
因此，每个插入的扫描不仅是扫描到子地图的当前最佳位置，它还是测距数据中可能错过其他可能性的一种表示。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.submaps.num_range_data

.. note::

    该值表示每个子地图中要存储的扫描数。
    一旦子地图中的扫描数量达到该值，它就会关闭并不会插入更多扫描，而是将新扫描插入一个新子地图。

.. note::

    我们发现一个较小的值会导致较短时间内较高频率的重建和更高的计算成本，但也能较快地适应环境变化。
    较大的值则在动态环境中表现不佳，但对于大范围的静态环境，可以减少计算成本。

全局 SLAM
--------

一旦本地 SLAM 生成了新的子地图，它的工作就完成了。
但是，由于本地 SLAM 无法纠正自身的长期漂移，Cartographer 必须在全局范围内工作以建立闭环。
全局 SLAM 会试图在不连续的轨迹中找到匹配，以便创建一个闭环。
这通过定期尝试将最近生成的子地图与之前生成的子地图进行匹配来实现。

首先，Cartographer 尝试将最新的子地图与之前的子地图进行 **局部闭环**。
局部闭环仅尝试与时间较近的子地图进行匹配，假设这些子地图在位置上非常接近。
这可以通过在每次生成新子地图时，尝试将新子地图与先前生成的若干个子地图进行匹配来实现。

.. code-block:: lua

    POSE_GRAPH.optimize_every_n_nodes

如果新子地图无法与任何之前生成的子地图匹配，Cartographer 将尝试 **全局闭环**。
全局闭环会将新子地图与所有其他子地图进行匹配，这样即使机器人回到了之前经过的地方，也可以识别出闭环。
由于全局闭环计算量大，因此 Cartographer 只会在指定数量的节点之后才尝试全局闭环。

.. code-block:: lua

    POSE_GRAPH.global_sampling_ratio

为了使全局 SLAM 有效，Cartographer 会在后台线程中定期尝试优化整个轨迹。
通过这样做，Cartographer 可以不断改进其对机器人位置的估计，并修正长期漂移。

.. code-block:: lua

    POSE_GRAPH.constraint_builder.sampling_ratio

优化全局轨迹的次数可以通过调整以下参数来控制：

.. code-block:: lua

    POSE_GRAPH.optimize_every_n_nodes

全局 SLAM 的最终目标是使机器人轨迹尽可能一致，并生成全局一致的地图。
这可以通过结合所有传感器数据，并尝试找到与所有子地图的一致性来实现。

总结
----

Cartographer 是一个强大的 SLAM 库，它结合了本地和全局 SLAM 技术，以生成高质量的地图。
通过调整不同的配置参数，可以根据具体的机器人和传感器设置优化 Cartographer 的性能。
本文概述了 Cartographer 的关键子系统及其配置选项，并提供了如何调整这些选项以改进 SLAM 性能的建议。
