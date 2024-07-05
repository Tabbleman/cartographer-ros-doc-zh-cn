# 编译 Cartographer ROS

## 系统要求

Cartographer ROS 的系统要求与[Cartographer 的系统要求](https://google-cartographer.readthedocs.io/en/latest/#system-requirements)相同。

当前支持以下[ROS 发行版](http://wiki.ros.org/Distributions)：

* Melodic
* Noetic

## 构建与安装

为了构建 Cartographer ROS，我们推荐使用 [`wstool`](http://wiki.ros.org/wstool) 和 [`rosdep`](http://wiki.ros.org/rosdep)。为了更快的构建速度，我们还推荐使用 [`Ninja`](https://ninja-build.org)。

在使用 ROS Noetic 的 Ubuntu Focal 系统上，使用以下命令安装上述工具：

```bash
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
```

在旧版本发行版上：

```bash
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build stow
```

安装工具后，在 `catkin_ws` 中创建一个新的 `cartographer_ros` 工作空间：

```bash
mkdir catkin_ws
cd catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
```

现在你需要安装 `cartographer_ros` 的依赖项。首先，我们使用 `rosdep` 来安装所需的包。如果你已经执行过 `sudo rosdep init` 命令，那么它会打印一个错误，可以忽略这个错误。

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

Cartographer 使用 `abseil-cpp` 库，需要通过以下脚本手动安装：

```bash
src/cartographer/scripts/install_abseil.sh 
```

由于版本冲突，你可能需要卸载 ROS 的 `abseil-cpp`：

```bash
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp 
```

构建并安装：

```bash
catkin_make_isolated --install --use-ninja
```

这将从主分支的最新版本构建 Cartographer。如果你需要特定版本，则需要在 `cartographer_ros.rosinstall` 中更改版本。

