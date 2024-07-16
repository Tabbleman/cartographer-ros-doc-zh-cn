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

.. _assets_writer:

利用 Cartographer ROS 生成的地图
=================================

随着传感器数据的输入，像 Cartographer 这样的 SLAM 算法的状态会不断演变，以保持*当前对机器人轨迹和周围环境的最佳估计*。
因此，Cartographer 能够提供的最精确的定位和制图结果是在算法完成时获得的。
Cartographer 可以将其内部状态序列化为 ``.pbstream`` 文件格式，该文件本质上是一个压缩的 protobuf 文件，包含了 Cartographer 内部使用的数据结构的快照。

为了在实时运行时保持高效，Cartographer 会立即丢弃大部分传感器数据，只使用其输入的小部分进行工作，内部使用的映射（并保存在 ``.pbstream`` 文件中）因此非常粗糙。
然而，当算法完成并建立最佳轨迹后，它可以与完整的传感器数据重新组合 *事后* 创建高分辨率地图。

Cartographer 通过 ``cartographer_assets_writer`` 实现了这种重组。
assets writer 接受以下输入

1. 用于执行 SLAM 的原始传感器数据（在 ROS ``.bag`` 文件中），
2. 在此传感器数据上执行 SLAM 时捕获的 cartographer 状态（保存在 ``.pbstream`` 文件中），
3. 传感器外参（即来自 bag 的 TF 数据或 URDF 描述），
4. 和管道配置，该配置在 ``.lua`` 文件中定义。

assets writer 使用在 ``.pbstream`` 中找到的轨迹分批运行 ``.bag`` 数据。
该管道可用于着色、过滤和导出 SLAM 点云数据为各种格式。
在管道中可以交错多种此类点处理步骤 - `cartographer/io`_ 中已经有几个可用。

示例用法
--------

当使用离线节点运行 Cartographer 时，会自动保存一个 ``.pbstream`` 文件。
例如，使用 3D 背包示例：

.. code-block:: bash

   wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-14-14-00.bag
   ros2 launch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag

在命令行上查看输出，直到节点终止。
它将写入 ``b3-2016-04-05-14-14-00.bag.pbstream``，该文件表示 Cartographer 在处理所有数据并完成所有优化后的状态。

当作为在线节点运行时，Cartographer 不知道你的 bag（或传感器输入）何时结束，因此你需要使用公开的服务来显式完成当前轨迹并让 Cartographer 序列化其当前状态：

.. code-block:: bash

   # 完成第一个轨迹。不会接受进一步的数据。
   ros2 service call /finish_trajectory 0

   # 请求 Cartographer 序列化其当前状态。
   # （按 tab 键快速展开参数语法）
   ros2 service call /write_state "{filename: '${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream', include_unfinished_submaps: "true"}"

一旦你获取了 ``.pbstream`` 文件，就可以使用 3D 背包的 `示例管道`_ 运行 assets writer：

.. _示例管道: https://github.com/cartographer-project/cartographer_ros/blob/44459e18102305745c56f92549b87d8e91f434fe/cartographer_ros/configuration_files/assets_writer_backpack_3d.lua

.. code-block:: bash

   ros2 launch cartographer_ros assets_writer_backpack_3d.launch \
      bag_filenames:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag \
      pose_graph_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream

所有输出文件的前缀为 ``--output_file_prefix``，默认值为第一个 bag 的文件名。
对于上一个示例，如果你在管道配置文件中指定了 ``points.ply``，这将转化为 ``${HOME}/Downloads/b3-2016-04-05-14-14-00.bag_points.ply``。

配置
-----

assets writer 被建模为 `PointsProcessor`_ 步骤的管道。
`PointsBatch`_ 数据流经每个处理器，它们都可以在传递之前修改 ``PointsBatch``。

.. _PointsProcessor: https://github.com/cartographer-project/cartographer/blob/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io/points_processor.h
.. _PointsBatch: https://github.com/cartographer-project/cartographer/blob/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io/points_batch.h

例如，`assets_writer_backpack_3d.lua`_ 管道使用 ``min_max_range_filter`` 去除离传感器太近或太远的点。
在此之后，它保存 "*X-Rays*"（地图的半透明侧视图），然后根据传感器帧 ID 重着色 ``PointsBatch``，并使用这些新颜色写入另一组 X-Rays。

.. _assets_writer_backpack_3d.lua: https://github.com/cartographer-project/cartographer_ros/blob/44459e18102305745c56f92549b87d8e91f434fe/cartographer_ros/configuration_files/assets_writer_backpack_3d.lua

可用的 ``PointsProcessor`` 全部定义在 `cartographer/io`_ 子目录中，并在各自的头文件中进行了文档说明。

.. _cartographer/io: https://github.com/cartographer-project/cartographer/tree/f1ac8967297965b8eb6f2f4b08a538e052b5a75b/cartographer/io

* **color_points**: 根据 frame_id 为点着固定颜色。
* **dump_num_points**: 通过点，但记录看到的点数并在 Flush 时输出。
* **fixed_ratio_sampler**: 仅让固定的 'sampling_ratio' 的点通过。'sampling_ratio' 为 1. 时，这个过滤器不起作用。
* **frame_id_filter**: 过滤掉具有黑名单 frame_id 或非白名单 frame id 的所有点。请注意，你可以指定白名单或黑名单，但不能同时指定。
* **write_hybrid_grid**: 使用 'voxel_size' 大小的体素创建混合网格。'range_data_inserter' 选项用于配置通过混合网格的范围数据射线追踪。
* **intensity_to_color**: 对来自 'frame_id' 传感器的每个点应用 ('intensity' - min) / (max - min) * 255，并用该值将点着色为灰色。如果 'frame_id' 为空，则适用于所有点。
* **min_max_range_filtering**: 过滤掉离其 'origin' 距离超过 'max_range' 或小于 'min_range' 的所有点。
* **voxel_filter_and_remove_moving_objects**: 体素过滤数据，并且仅传递我们认为在非移动物体上的点。
* **write_pcd**: 将 PCD 文件流式传输到磁盘。标头在 'Flush' 中写入。
* **write_ply**: 将 PLY 文件流式传输到磁盘。标头在 'Flush' 中写入。
* **write_probability_grid**: 使用指定的 'resolution' 创建概率网格。由于所有点都投影到 x-y 平面上，数据的 z 组件将被忽略。'range_data_inserter' 选项用于配置通过概率网格的范围数据射线追踪。
* **write_xray_image**: 使用 'voxel_size' 大小的像素创建点的 X 射线切片。
* **write_xyz**: 写入 ASCII xyz 点。

第一人称视角点云可视化
------------------------

两个特别有趣的 ``PointsProcessor`` 是：``pcd_writing`` 和 ``ply_writing``，它们可以将点云保存为 ``.pcd`` 或 ``.ply`` 文件格式。
这些文件格式可以被诸如 `point_cloud_viewer`_ 或 `meshlab`_ 之类的专业软件使用，以便在高分辨率地图中导航。

.. _point_cloud_viewer: https://github.com/cartographer-project/point_cloud_viewer
.. _meshlab: http://www.meshlab.net/

此结果的典型 assets writer 管道由一个 IntensityToColorPointsProcessor_ 组成，为点赋予非白色颜色，然后由一个 PlyWritingPointsProcessor_ 导出结果到 ``.ply`` 点云。
此类管道的示例在 `assets_writer_backpack_2d.lua`_ 中。

.. _IntensityToColorPointsProcessor: https://github.com/cartographer-project/cartographer/blob/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io/intensity_to_color_points_processor.cc
.. _PlyWritingPointsProcessor: https://github.com/cartographer-project/cartographer/blob/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io/ply_writing_points_processor.h
.. _assets_writer_backpack_2d.lua: https://github.com/cartographer-project/cartographer_ros/blob/44459e18102305745c56f92549b87d8e91f434fe/cartographer_ros/configuration_files/assets_writer_backpack_2d.lua

一旦你有了 ``.ply`` 文件，请按照 `point_cloud_viewer`_ 的 README 生成一个磁盘上的八叉树数据结构，该数据结构可以由同一个 repo 中的查看器（基于 SDL 或 Web）查看。
请注意，color 是 ``point_cloud_viewer`` 功能所必需的。

.. _point_cloud_viewer: https://github.com/cartographer-project/point_cloud_viewer

.. image:: point_cloud_viewer_demo_3d.jpg
