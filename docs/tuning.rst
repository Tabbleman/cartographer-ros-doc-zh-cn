.. 版权所有 2018 The Cartographer Authors

.. 根据 Apache 许可证，版本 2.0（以下简称“许可证”）许可；
   除非符合许可证，否则您不得使用此文件。
   您可以在以下地址获取许可证副本：

..      http://www.apache.org/licenses/LICENSE-2.0

.. 除非适用法律要求或书面同意，按许可证分发的软件
   是“按原样”分发的，不提供任何明示或暗示的担保或条件。
   有关许可证下权限和限制的具体语言，请参阅许可证。

.. cartographer SHA: aba4575d937df4c9697f61529200c084f2562584
.. cartographer_ros SHA: 99c23b6ac7874f7974e9ed808ace841da6f2c8b0
.. TODO(hrapp): 在某处提到 insert_free_space

调优方法
========

不幸的是，调整 Cartographer 真的很难。
系统有许多参数，其中许多参数相互影响。
本调优指南试图通过具体示例解释一种原则性方法。

内置工具
--------

Cartographer 提供内置的 SLAM 评估工具，这些工具对于测量局部 SLAM 质量特别有用。
它们是独立的可执行文件，随核心 `cartographer` 库一起提供，因此是独立的，但与 cartographer_ros 兼容。
因此，请访问 Cartographer Read the Docs 评估网站_，获取概念概述和如何在实践中使用这些工具的指南。

这些工具假定您已将 SLAM 状态序列化为 `.pbstream` 文件。
使用 `cartographer_ros`，您可以调用 assets_writer 序列化状态 - 有关详细信息，请参阅 :ref:`assets_writer` 部分。

.. _Cartographer Read the Docs 评估网站: https://google-cartographer.readthedocs.io/en/latest/evaluation.html

示例：调优局部 SLAM
-------------------

在本示例中，我们将从 `cartographer` 提交记录 aba4575_ 和 cartographer_ros 提交记录 99c23b6_ 开始，并查看我们的测试数据集中的 bag b2-2016-04-27-12-31-41.bag。

在我们的初始配置中，我们很早就看到了袋子中的一些滑动现象。
背包经过了德国博物馆的一个坡道，这违反了平地的 2D 假设。
在激光扫描数据中可以看到，矛盾的信息被传递给了 SLAM。
但滑动现象也表明我们过于信任点云匹配，忽略了其他传感器。
我们的目标是通过调优来改善这种情况。

.. _aba4575: https://github.com/cartographer-project/cartographer/commit/aba4575d937df4c9697f61529200c084f2562584
.. _99c23b6: https://github.com/cartographer-project/cartographer_ros/commit/99c23b6ac7874f7974e9ed808ace841da6f2c8b0

如果我们只看这个特定的子图，错误完全包含在一个子图中。
我们还看到，随着时间的推移，全球 SLAM 发现发生了一些奇怪的事情并部分修正了它。
损坏的子图将永远损坏。

.. TODO(hrapp): 视频

由于这里的问题是子图内的滑动，这是一个局部 SLAM 问题。
所以让我们关闭全球 SLAM，以免干扰我们的调优。

.. code-block:: lua

   POSE_GRAPH.optimize_every_n_nodes = 0

正确的子图大小
^^^^^^^^^^^^^^^^

子图的大小通过 `TRAJECTORY_BUILDER_2D.submaps.num_range_data` 配置。
查看此示例的各个子图，它们已经很好地符合两个约束，因此我们认为该参数已被很好地调优。

调优 `CeresScanMatcher`
^^^^^^^^^^^^^^^^^^^^^^^

在我们的案例中，扫描匹配器可以自由地前后移动匹配，而不会影响得分。
我们希望通过让扫描匹配器在偏离其获得的先验时付出更多代价来惩罚这种情况。
控制这两个参数的是 `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight` 和 `rotation_weight`。
值越高，从先验中移动结果的代价就越大，换句话说：扫描匹配必须在另一个位置生成更高的得分才能被接受。

为了教学目的，让我们让偏离先验的代价变得非常高：

.. code-block:: lua

   TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e3

.. TODO(hrapp): 视频

这允许优化器非常自由地覆盖扫描匹配器结果。
这导致接近先验的姿态，但与深度传感器不一致并且显然是错误的。
尝试该值在 `2e2` 时产生了更好的结果。

.. TODO(hrapp): 视频，translation_weight = 2e2

在这里，扫描匹配器仍然使用旋转稍微搞砸了结果。
将 `rotation_weight` 设置为 4e2 后，我们得到了合理的结果。

验证
^^^^

为了确保我们没有针对这个特定问题过度调优，我们需要对收集的其他数据运行配置。
在这种情况下，新参数确实揭示了滑动现象，例如在 `b2-2016-04-05-14-44-52.bag` 的开头，因此我们不得不将 translation_weight 降低到 1e2。
此设置对于我们想要修复的情况较差，但不再滑动。
在检查它们之前，我们会归一化所有权重，因为它们只有相对意义。
本次调优的结果是 PR428_。
通常，始终尝试针对平台进行调优，而不是特定的 bag。

.. _PR428: https://github.com/cartographer-project/cartographer/pull/428

特殊情况
---------

默认配置和上述调优步骤集中于质量。
只有在我们获得良好质量之后，我们才能进一步考虑特殊情况。

低延迟
^^^^^^

低延迟是指在接收到传感器输入后不久，通常在一秒钟内，优化的局部姿态变得可用，并且全局优化没有积压。
低延迟是在线算法（例如机器人定位）所必需的。
局部 SLAM 在前台操作，直接影响延迟。
全局 SLAM 构建后台任务队列。
当全局 SLAM 无法跟上队列时，漂移可能无限期积累，因此全局 SLAM 应该调优以实时工作。

有许多选项可以调优不同组件的速度，我们按推荐的、直接的顺序列出它们到更具侵入性的选项。
建议一次只探索一个选项，从第一个开始。
配置参数在 `Cartographer文档`_ 中有记录。

.. _Cartographer文档: https://google-cartographer.readthedocs.io/en/latest/configuration.html

为了调优全局 SLAM 以降低延迟，我们减少其计算负载，直到它始终跟上实时输入。
在此阈值下，我们不再进一步减少，但尝试获得尽可能好的质量。
为了降低全局 SLAM 延迟，我们可以

- 减少 `optimize_every_n_nodes`
- 增加 `MAP_BUILDER.num_background_threads` 直到核心数
- 减少 `global_sampling_ratio`
- 减少 `constraint_builder.sampling_ratio`
- 增加 `constraint_builder.min_score`
- 对于自适应体素滤波器，减少 `.min_num_points`、.max_range，增加 .max_length
- 增加 `voxel_filter_size`、submaps.resolution，减少 submaps.num_range_data
- 减小搜索窗口大小，`.linear_xy_search_window`、`.linear_z_search_window`、`.angular_search_window`
- 增加 `global_constraint_search_after_n_seconds`
- 减少 `max_num_iterations`

为了调优局部 SLAM 以降低延迟，我们可以

- 增加 `voxel_filter_size`
- 增加 `submaps.resolution`
- 对于自适应体素滤波器，减少 `.min_num_points`、.max_range，增加 .max_length
- 减少 `max_range`（特别是如果数据嘈杂）
- 减少 `submaps.num_range_data`

请注意，较大的体素会稍微增加扫描匹配得分，因此应相应地增加得分阈值。

在给定地图中的纯定位
^^^^^^^^^^^^^^^^^^^^^^^^^^

纯定位不同于制图。
首先，我们希望局部和全局 SLAM 的延迟较低。
其次，全局 SLAM 通常会在作为地图的冻结轨迹和当前轨迹之间找到大量内部约束。

为了调优纯定位，我们应首先启用 `TRAJECTORY_BUILDER.pure_localization = true` 并
大幅减少 `POSE_GRAPH.optimize_every_n_nodes` 以获得频繁的结果。
在这些设置下，全局 SLAM 通常会太慢，无法跟上。
下一步，我们大幅减少 `global_sampling_ratio` 和 `constraint_builder.sampling_ratio`
以补偿大量约束。
然后我们按照上述方法调优以降低延迟，直到系统可靠地实时工作。

如果你在 `pure_localization` 中运行，submaps.resolution **应该匹配** .pbstream 中运行的子图的分辨率。
目前尚未测试使用不同分辨率，可能无法按预期工作。

全局优化中的里程计
^^^^^^^^^^^^^^^^^^^^^^

如果单独的里程计源用作局部 SLAM (`use_odometry = true) 的输入，我们也可以调优全局 SLAM 以受益于此额外信息。

总共有四个参数可以让我们调优局部 SLAM 和里程计在优化中的单独权重：

.. code-block:: lua

    POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight
    POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight
    POSE_GRAPH.optimization_problem.odometry_translation_weight
    POSE_GRAPH.optimization_problem.odometry_rotation_weight

我们可以根据我们对局部 SLAM 或里程计的信任程度设置这些权重。
默认情况下，里程计以类似于局部 SLAM（扫描匹配）姿态的方式加权到全局优化中。
但是，来自轮编码器的里程计通常在旋转方面有很高的不确定性。
在这种情况下，可以减少旋转权重，甚至降至零。

仍有问题？
----------

cartographer 已经停止维护啦，只能求助身边懂的人了！

.. note::

   已经有很多 GitHub 问题，开发人员解决了各种问题。浏览 cartographer_ros_ 和 cartographer_ 的已解决问题是了解 Cartographer 的好方法，或许可以找到您问题的解决方案！

.. _cartographer_ros 的已解决问题: https://github.com/cartographer-project/cartographer_ros/issues?q=is%3Aissue+is%3Aclosed
.. _cartographer: https://github.com/cartographer-project/cartographer_ros/issues?q=is%3Aissue+is%3Aclosed
