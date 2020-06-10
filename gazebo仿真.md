

**安装完整版的ROS。**

**安装一些ROS-Gazebo组件**

```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```

**安装turtlebot相关包**

```
 sudo apt-get install ros-kinetic-turtlebot-*
```

**启动Gazebo并加载机器人、环境模型**

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```

**启动键盘遥控节点**

```
roslaunch turtlebot_teleop keyboard_teleop.launch --screen
```

**运行cartographer   rviz  **

