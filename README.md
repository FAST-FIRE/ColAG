# ColAG

**ColAG:** A **Col**laborative **A**ir-**G**round Framework for Perception-Limited UGVs' Navigation

## Table of Contents

1. [About](#1-about)
2. [How to Use](#2-how-to-use)
3. [Acknowledgments](#3-acknowledgments)
4. [License](#4-license)
5. [Maintenance](#5-maintenance)

## 1. About

**Author**: Zhehan Li $^\dagger$, Rui Mao $^\dagger$, Nanhe Chen, Chao Xu, Fei Gao, Yanjun Cao *

**Preprint Paper**: [ColAG: A Collaborative Air-Ground Framework for Perception-Limited UGVs' Navigation](https://arxiv.org/abs/2310.13324).

Accepted in [ICRA2024](https://2024.ieee-icra.org/).

```bib
@article{li2023colag,
  title={ColAG: A Collaborative Air-Ground Framework for Perception-Limited UGVs' Navigation},
  author={Li, Zhehan and Mao, Rui and Chen, Nanhe and Xu, Chao and Gao, Fei and Cao, Yanjun},
  journal={arXiv preprint arXiv:2310.13324},
  year={2023}
}
```

If our source code is used in your academic projects, please cite our paper. Thank you!

## 2. How to Use

Compiling tests passed on Ubuntu 20.04 with ros1 installed.

- Follow the [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- Follow the installation of [MARSIM](https://github.com/hku-mars/MARSIM)
- Follow the installation of [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
- Follow the installation of [OR-Tools](https://github.com/google/or-tools), use the Python version
- Follow the installation of [diablo_mpc](https://github.com/GaoLon/diablo_mpc)

For convenience, we listed the installation steps here

```sh
sudo apt install libglfw3-dev libglew-dev libarmadillo-dev libzmqpp-dev ros-noetic-mavros

git clone https://github.com/osqp/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" .. 
cmake --build .
sudo cmake --build . --target install
# if meet "CMake 3.18 or higher is required. 
# You are running version 3.16.3", 
# change "cmake_minimum_required(VERSION 3.18)" 
# to "cmake_minimum_required(VERSION 3.16)"
# in all CMakeLists.txt.

git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build
cd build
cmake ..
make
sudo make install
# if meet error, may caused by version mismatch
# we backup osqp and osqp-eigen source code in folder .bak,
# which is tested on Ubuntu 20.04

sudo apt install python3-pip
python3 -m pip install --upgrade --user ortools
```

The Air_ws, Ground_ws, MARSIM_ws are separated catkin workspace, since they based on ego-swarm

```sh
cd MARSIM_ws
catkin_make
```

```sh
cd Air_ws
catkin_make
```

```sh
cd Ground_ws
catkin_make
```

Open rviz for visualization

```sh
cd Air_ws
source devel/setup.sh
roslaunch ego_planner rviz.launch
```

Run the following in different command windows

Ensure the `ugv_num` is same in both swarm_sim.launch

```sh
cd MARSIM_ws
source devel/setup.sh
roslaunch test_interface single_drone_vlp32.launch
```

```sh
cd Ground_ws
source devel/setup.sh
roslaunch ego_planner swarm_sim.launch ugv_num:=3
```

```sh
cd Air_ws
source devel/setup.sh
roslaunch ego_planner swarm_sim.launch ugv_num:=3
```

Or use the [run.sh](run.sh) to run the launches above

```sh
./run.sh 3 # the ugv num
```

Then send a trigger to start blind navigation

```sh
rostopic pub /traj_start_trigger geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```

## 3. Acknowledgments

**There are several important works which support this project:**

- [MARSIM](https://github.com/hku-mars/MARSIM): A lightweight point-realistic simulator for LiDAR-based UAVs.
- [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm): A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments.
- [OR-Tools](https://github.com/google/or-tools): Google's software suite for combinatorial optimization.
- [diablo_mpc](https://github.com/GaoLon/diablo_mpc): A MPC for diablo configuration(see as differential car).

## 4. License

The source code is released under the [GPLv3](https://www.gnu.org/licenses/) license.

## 5. Maintenance

We are still working on extending the proposed system and improving code reliability.

For any technical issues, please contact Zhehan Li (<zhehanli@zju.edu.cn>) or Yanjun Cao (<yanjunhi@zju.edu.cn>).

For commercial inquiries, please contact Yanjun Cao (<yanjunhi@zju.edu.cn>).
