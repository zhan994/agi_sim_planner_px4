# agi_sim_planner_px4

**A repo. which used to check PnC algorithms using PX4-Gazebo simulation tools.**

## Prerequisites

- ros1 noetic
- [px4_firmware](https://docs.px4.io/main/en/ros/mavros_installation.html)
- QGC
- nlopt



## Run

Copy data in folder `sim_tools` to **px4_firmware**.

```bash
git clone git@github.com:zhan994/agi_sim_px4_ros1.git

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

Build `nlopt`.

```bash
cd third/nlopt
mkdir build && cmake ..
make
sudo make install
```
