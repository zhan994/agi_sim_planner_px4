# agi_sim_planner_px4

**A repo. which used to check PnC algorithms using PX4-Gazebo simulation tools.**

## Prerequisites

- ros1 noetic
- [px4_firmware](https://docs.px4.io/main/en/ros/mavros_installation.html)
- QGC
- nlopt

Baidu NetDisk for px4_firmware :  https://pan.baidu.com/s/10zeMxHhbHWKQPqyAnBEwAg?pwd=vpxw 

You can add sth. from [XTDrone](https://gitee.com/robin_shaun/XTDrone.git) into px4_firmware.

```bash
git clone https://gitee.com/robin_shaun/XTDrone.git
cd XTDrone
git checkout 1_13_2
git submodule update --init --recursive

# 修改启动脚本文件
cp sitl_config/init.d-posix/* <path-to-px4_firmware>/ROMFS/px4fmu_common/init.d-posix/

# 添加launch文件
cp -r sitl_config/launch/* <path-to-px4_firmware>/launch/

# 添加世界文件
cp sitl_config/worlds/* <path-to-px4_firmware>/Tools/sitl_gazebo/worlds/

# 修改部分插件
cp sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.cpp <path-to-px4_firmware>/Tools/sitl_gazebo/src
cp sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.hh <path-to-px4_firmware>/Tools/sitl_gazebo/include
cp sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.cpp <path-to-px4_firmware>/Tools/sitl_gazebo/src
cp sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.h <path-to-px4_firmware>/Tools/sitl_gazebo/include

# 修改CMakeLists.txt
cp sitl_config/CMakeLists.txt <path-to-px4_firmware>/Tools/sitl_gazebo

# 修改部分模型文件
cp -r sitl_config/models/* <path-to-px4_firmware>/Tools/sitl_gazebo/models/ 

# 删除系统中的同名文件
cd ~/.gazebo/models/
rm -r stereo_camera/ 3d_lidar/ 3d_gpu_lidar/ hokuyo_lidar/

# 重新编译
cd <path-to-px4_firmware>
rm -r build/
make px4_sitl_default gazebo
```

## Run

Copy data in folder `sim_tools` to **px4_firmware**.

```bash
git clone git@github.com:zhan994/agi_sim_planner_px4.git

cd agi_sim_px4_ros1/sim_tools
# 添加launch文件
cp launch/* ~/work/px4_firmware/launch/
# 添加世界文件
cp worlds/* ~/work/px4_firmware/Tools/sitl_gazebo/worlds/
# 修改部分模型文件
cp -r models/* ~/work/px4_firmware/Tools/sitl_gazebo/models/ 
```

Move `QGC` to `px4_firmware` folder, and run scipts.

```bash
source scripts/px4_env.sh
./scripts/qgc.sh
roslaunch px4 demo1.launch
```

## Fast-Planner

Build `nlopt`.

```bash
cd third/nlopt
mkdir build && cmake ..
make
sudo make install

cd fast_planner
catkin_make
source devel/setup.bash
./fast.sh
```

## EGO-Planner-V2

```bash
cd ego_planner_v2
catkin_make
source devel/setup.bash
./ego.sh
```
