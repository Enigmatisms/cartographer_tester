# Cartographer_tester

---

This repo originated from [Google/cartographer_ros](https://github.com/cartographer-project/cartographer_ros). Technically and legally, I don't own this repo because most of the work in here is not mine. I only contributed a little bit (listed below). The original repo only contains its ros implementation. To run the whole SLAM project, we need [Google/cartographer](https://github.com/cartographer-project/cartographer). It can be considered that [cartographer] and [cartographer_ros] are two "ros packages" in a catkin workspace (actually, cartographer itself is not), which depends on each other during compilation.

I added some utilities inside this repo, for experiment purposes:

- Off-line node modification, after the rosbag play is completed, off-line node detects the idleness of itself. Being idle after certain time will trigger trajectory output (first to pbstream file and then extracted to .tjc file (tjc is a format specified by me).
- Automatic cartographer runner. It is able to run multiple rosbags sequentially (and save their output trajectories). Automatic and convenient, implemented by shell.
  - `automatic.sh`: running bags specifying the location (folder) which contains the bags
  - `automatic_file.sh`: running bags of which names are listed in a `.conf` file.

---

## Compilation

I combined those two repos, therefore compilation would be easy on Ubuntu 18.04 with ros melodic:

```shell
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
# install ceres-solver (dependency), I use ceres-2.0.0 and it compiles.
src/cartographer/scripts/install_abseil.sh		# installing abseil from source
sudo chmod +x ./make.sh
./make.sh
```

---

## Problems

For some of the problems that could be encountered during compilation (like missing Lua or ABSL error, etc.), you can refer to [(Chinese Blog)Cartographer编译问题整理](https://enigmatisms.github.io/2022/04/11/Cartographer%E7%BC%96%E8%AF%91%E9%97%AE%E9%A2%98%E6%95%B4%E7%90%86/) of mine for some inspiration.
