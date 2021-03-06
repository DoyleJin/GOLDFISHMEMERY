# **Cartographer安装**

参考：https://doc.bwbot.org/zh-cn/books-online/xq-manual/topic/136.html

# 1.安装依赖包

```bash
sudo apt-get update
sudo apt-get install -y \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libprotobuf-dev \
    libsuitesparse-dev \
    libwebp-dev \
    ninja-build \
    protobuf-compiler \
    python-sphinx
```

# 2．建ros工作空间

```bash
mkdir catkin_goole_ws ##cartographer工作空间
cd catkin_goole_ws
mkdir Relatedprogram   ##安装相关库（ceres和prtobuf）的安装包位置,便于文件查找和管理
```

# 3．安装ceres solver

（来自谷歌的非线性优化库）

参考博文https://blog.csdn.net/weixin_39373577/article/details/81285420

官网http://www.ceres-solver.org/installation.html

## 3.1下载

```bash
cd Relatedprogram 
mkdir ceres-solver
cd ceres-solver
git clone https://github.com/ceres-solver/ceres-solver.git
##git clone https://ceres-solver.googlesource.com/ceres-solver
```

如需要解压(如果是用第一个网址下载的，不用解压)

```bash
tar -xzxf ceres-solver-××××××××
```

## 3.2 ceres是google库，安装相关依赖

```bash
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev
sudo apt-get install libgoogle-glog-dev libgtest-dev
```

如果安装时找不到 cxsparse 或者其他的lib，需要添加下面的源

```bash
sudo gedit /etc/apt/sources.list
```

​		把下面的源粘贴到source.list的最上方 

```bash
deb http://cz.archive.ubuntu.com/ubuntu trusty main universe
```

​		更新一下 

```bash
sudo apt-get update
```

## 3.3.编译和安装ceres

```bash
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j3
make test
sudo make install
```

用make -j带一个参数，可以把项目在进行并行编译，比如在一台双核的机器上，完全可以用make -j4，让make最多允许4个编译命令同时执行，这样可以更有效的利用CPU资源。并不是j后面加的数字越大越好，还要看文件支不支持多线程编译。

# 4. 安装prtobuf 3.0

protocol buffers 是一种语言无关、平台无关、可扩展的序列化结构数据的方法，它可用于（数据）通信协议、数据存储等

```bash
cd ..
cd .. ##进入Relatedprogram目录
git clone https://github.com/google/protobuf.git
cd protobuf
git checkout v3.6.1
mkdir build
cd build
cmake  \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  ../cmake
make -j4
sudo make install
```

# 5. 安装cartographer

```bash
cd ..
cd .. ##进入Relatedprogram目录
git clone https://github.com/BlueWhaleRobot/cartographer.git
cd cartographer
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

​		试了一下j4， 并没有感觉编译变很快

# 6．安装cartographer_ros

```bash
cd
cd catkin_goole_ws#请修改路径到自己的ROS catkin工作空间
wstool init src
cd src   
git clone https://github.com/BlueWhaleRobot/cartographer_ros.git
cd ..
catkin_make_isolated --install --use-ninja

######!!!!!!!不要用catkin_make，之后会碰到各种错
```

# 7．安装完成，下载测试用的bag文件

点击下述链接下载文件，保存到桌面

https://www.bwbot.org/s/vQ2D9Z

# 8．启动demo演示

正常可以看到rviz启动并开始建图,根据个人平台计算能力不同，本demo完整运行时间一般为半个小时到1个小时之间(实际测试2个小时)

```bash
source install_isolated/setup.bash
####source catkin_goole_ws/devel/setup.bash

roslaunch cartographer_ros offline_backpack_2d.launch  bag_filenames:=${HOME}/Desktop/cartographer_paper_deutsches_museum.bag
```


3d demo 

```bash
roslaunch cartographer_ros offline_backpack_3d.launch  bag_filenames:=${HOME}/Desktop/b3-2016-04-05-14-14-00.bag
```

# 9．提取建立的地图，结束测试

完成轨迹, 不接受进一步的数据。
```
rosservice call /finish_trajectory 0
```
序列化保存其当前状态
```
rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"
```
将pbstream转换为pgm和yaml
```
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/mymap -pbstream_filename=${HOME}/Downloads/mymap.pbstream -resolution=0.05
```

现在在home目录下的Downloads文件夹内会生成建立的地图文件，这两个文件（pgm 和 yaml）在ros中的map_server中可以加载使用





