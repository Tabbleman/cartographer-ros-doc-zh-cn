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

======================================
运行Cartographer ROS在示例包上
======================================

现在Cartographer和Cartographer的ROS集成已经安装，你可以下载示例包（例如2D和3D背包采集的`德意志博物馆 <https://en.wikipedia.org/wiki/Deutsches_Museum>`_）到已知位置，在此例中是``~/Downloads``，然后使用``roslaunch``启动演示。

启动文件会自动启动``roscore``和``rviz``。

.. warning:: 当你想运行cartographer_ros时，你可能需要先通过运行``source install_isolated/setup.bash``来配置你的ROS环境（如果你的shell是zsh则替换为zsh）。

德意志博物馆
================

下载并启动2D背包演示：

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
    roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag

下载并启动3D背包演示：

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
    roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag

纯定位
=================

纯定位使用2个不同的包。第一个用于生成地图，第二个用于运行纯定位。

下载来自德意志博物馆的2D包：

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-05-14-44-52.bag
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-27-12-31-41.bag

生成地图（等待cartographer_offline_node完成）然后运行纯定位：

.. code-block:: bash

    roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag
    roslaunch cartographer_ros demo_backpack_2d_localization.launch \
       load_state_filename:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag.pbstream \
       bag_filename:=${HOME}/Downloads/b2-2016-04-27-12-31-41.bag

下载来自德意志博物馆的3D包：

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-13-54-42.bag
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-15-52-20.bag

生成地图（等待cartographer_offline_node完成）然后运行纯定位：

.. code-block:: bash

    roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag
    roslaunch cartographer_ros demo_backpack_3d_localization.launch \
       load_state_filename:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag.pbstream \
       bag_filename:=${HOME}/Downloads/b3-2016-04-05-15-52-20.bag

静态地标
================

  .. raw:: html

      <iframe width="560" height="315" src="https://www.youtube.com/embed/E2-OD-ycivc" frameborder="0" allowfullscreen></iframe>

  .. code-block:: bash

    # 下载地标示例包。
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/mir/landmarks_demo_uncalibrated.bag

    # 启动地标演示。
    roslaunch cartographer_mir offline_mir_100_rviz.launch bag_filename:=${HOME}/Downloads/landmarks_demo_uncalibrated.bag

Revo LDS
========

下载并启动从Neato Robotics真空吸尘器中的低成本Revo激光测距传感器捕获的示例包：

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/revo_lds/cartographer_paper_revo_lds.bag
    roslaunch cartographer_ros demo_revo_lds.launch bag_filename:=${HOME}/Downloads/cartographer_paper_revo_lds.bag

PR2
===

下载并启动从Willow Garage的PR2研发人形机器人捕获的示例包：

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/pr2/2011-09-15-08-32-46.bag
    roslaunch cartographer_ros demo_pr2.launch bag_filename:=${HOME}/Downloads/2011-09-15-08-32-46.bag

Taurob Tracker
==============

下载并启动从Taurob Tracker远程操作机器人捕获的示例包：

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/taurob_tracker/taurob_tracker_simulation.bag
    roslaunch cartographer_ros demo_taurob_tracker.launch bag_filename:=${HOME}/Downloads/taurob_tracker_simulation.bag
