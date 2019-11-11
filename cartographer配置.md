# cartographer 配置

------

#### ***！！！！！一定按照catkin_make_isolated --install --use-ninja编译工作空间！！！！***

之后 

```bash
source install_isolated/setup.bash
```

或者打开 .bashrc 文件把

```bash
source ~/catkin_goole_ws/install_isolated/setup.bash --extend
```

放入文件尾



### !!!!先启动机器人，后启动ｃａｒｔｏｇｒａｐｈｅｒ节点。而且关闭该节点后不能二次启动，必须与机器人启动文件一起启动

------

## **1.　launch文件中参数配置**

### 1.1　launch文件位置

由于source install_isolated/setup.bash，所以在src中修改launch文件后需要重新编译

```bash
/home/doyel/catkin_goole_ws/install_isolated/share/cartographer_ros/launch/aboutsuccess.launch
```

### 1.2　launch文件内容：

```xml
<launch>
  <!--param name="/use_sim_time" value="true" /这个配置是给仿真时候用的-->
  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename mapping.lua"
      output="screen">
 <!--  mapping.lua 作用是参数配置  -->
    <remap from="scan" to="f_scan_rep117" />
    <remap from="odom" to="odom_enc" />
    <remap from="imu" to="imu_data" />
  </node>
  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
    type="cartographer_occupancy_grid_node" args="-resolution 0.005" />
  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />
</launch>

```



### 1.3　功能：

主要修改：

```xml
  <remap from="scan" to="f_scan_rep117" />
  <remap from="odom" to="odom_enc" />
  <remap from="imu" to="imu_data" />
```

启动cartographer_node，加载 .lua配置文件

​	remap  雷达　里程计　IMU的话题到自己机器人发出的话题

启动绘图节点

启动可视化界面rviz

------

## 2 　.lua文件参数配置

配置参考

[ｃｓｄｎ]: https://blog.csdn.net/YiKangJ/article/details/88845663#_Cartographer_ROS__7
[官网options配置]: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html
[官网其余参数配置]: https://google-cartographer.readthedocs.io/en/latest/configuration.html

### 1.1　.lua文件位置

```bash
/home/doyel/catkin_goole_ws/install_isolated/share/cartographer_ros/configuration_files/aboutsuccess.lua
```



###　1.2　.lua文件内容

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  ---地图坐标系（固定fixed frame）                             ------tracking_frame = "laser_link",
  tracking_frame = "imu_link", ---baselink
  --- 2.tracking_frame=”imu_link",机器人中心坐标系，很重要，其它传感器数据都是以这个为基础进行插入的。cartographer_ros里面有个tf_bridge的类就是专门用来查询其它坐标系到此坐标系的转换关系的。
  -------published_frame = "laser_link",
  published_frame = "odom",---建图时候和map_frame是重合的
  odom_frame = "odom",---建图时候和map_frame是重合的(或者说有固定的坐标变换)
  provide_odom_frame = false,
  ----?????false
  publish_frame_projected_to_2d = false,
  -------use_odometry = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 28.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.

TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

return options

```

### 1.3　功能

将机器人的坐标与cartographer关联，设置相关参数

# 3. 编译

#### 1.将launch文件中引用的 lua的文件名改成自己的lua文件名

#### 2.编译（每次在src中修改完launch文件或者其他文件后，都需要重新编译才能生效）

```bash
cd 
cd catkin_goole_ws ##进入工作空间目录
catkin_make_isolated --install --use-ninja
```





# 4 一些参考博文



https://blog.csdn.net/flyinsilence/article/details/51854123

最常用的就是map，odom，base_link，base_laser坐标系，这也是开始接触gmapping的一些坐标系。

map:地图坐标系，顾名思义，一般设该坐标系为固定坐标系（fixed frame），一般与机器人所在的世界坐标系一致。

base_link:机器人本体坐标系，与机器人中心重合，当然有些机器人(PR 2)是base_footprint,其实是一个意思。

odom：里程计坐标系，这里要区分开odom  topic，这是两个概念，一个是坐标系，一个是根据编码器（或者视觉等）计算的里程计。但是两者也有关系，odom topic  转化得位姿矩阵是odom-->base_link的tf关系。这时可有会有疑问，odom和map坐标系是不是重合的？（这也是我写这个博客解决的主要问题）可以很肯定的告诉你，机器人运动开始是重合的。但是，随着时间的推移是不重合的，而出现的偏差就是里程计的累积误差。那map-->odom的tf怎么得到?就是在一些校正传感器合作校正的package比如amcl会给出一个位置估计（localization），这可以得到map-->base_link的tf，所以估计位置和里程计位置的偏差也就是odom与map的坐标系偏差。所以，如果你的odom计算没有错误，那么map-->odom的tf就是0.

base_laser:激光雷达的坐标系，与激光雷达的安装点有关，其与base_link的tf为固定的



https://blog.csdn.net/m0_37672916/article/details/77198261

\1. map_frame = “map”,cartographer中使用的全局坐标系，最好保持默认，否则ROS的Rviz不认识其它的定义，导致无法可视化建图过程。
 2.tracking_frame=”base_link”,机器人中心坐标系，很重要，其它传感器数据都是以这个为基础进行插入的。cartographer_ros里面有个tf_bridge的类就是专门用来查询其它坐标系到此坐标系的转换关系的。
 \3. published_frame = “base_link” 
 \4. odom_frame = “odom” ，3与4是配合使用的，如果参数provide_odom_frame = true  那么最后可视化时，发布的转换消息是从 published_frame->odom_frame->map_frame,  也即cartographer内部计算出了未经回环检测的局部图坐标到优化后的全局图坐标之间的转换关系并发布了出来。在跑官网的二维背包例子时，在map坐标周围一直跳动的odom就是这个玩意。



https://blog.csdn.net/u012686154/article/details/88174195

学ROS大半年了，之前一直没搞明白odom和map，今天重新查资料看博客加自己思考才真正理解了。深深怀疑自己的理解能力。。。

此处参考了https://blog.csdn.net/flyinsilence/article/details/51854123，博主的解释一开始我还是没太看明白，看了好几遍才看懂，懂了之后才发现人家写的真的是言简意赅啊！

我就详细的把自己的理解以及想到的例子写出来，万一有人也是不太明白这个呢。

举个栗子

比如说我现在算出来了一个一维的坐标**X=10(odom-->base_link)**，但是这个位置的真实坐标应该是**X'=2(map-->base_link)**，也就是说我计算的坐标和真实坐标出现了偏差。但是ROS在说的时候换了个说法，不说计算出来的坐标在map中漂了8，而是说是odom坐标系相对于map坐标系漂了8。所以此时base_link相对odom坐标是真实坐标x=2（因为odom坐标系短时间是准确的，如果是刚开始时odom和map重合，odom中的该坐标就是真实坐标）;odom相对于map坐标是**driftX=8(map-->odom)**;最后计算出来的base_link相对于map的坐标就是**X=drixfX+X'=10**了。

如果用IMU作积分的话，通过IMU获得的是odom坐标系下的坐标，初始时odom和map重合，都为0。那么短时间内，由于IMU的漂移很小，所以获得的位移deltaX是准确的，最终的在odom坐标系下的坐标是X=dextaX，在map中的坐标X'也是deltaX。但是时间长了之后，IMU积分开始产生漂移driftX了，最终在map下的坐标X=deltaX+driftX。由于我们定义IMU在短时间内产的的位移deltaX是准确的，所以deltaX是就是IMU在odom坐标系下的真实位移，但是此时odom相对于map则漂移了driftX。

 

所以odom相当于是根据**实际计算得到的坐标和真实坐标之间的变换**虚拟出来的一个坐标系。



https://blog.csdn.net/weixin_34055787/article/details/93835165

1. ROS中base_link, odom, fixed_frame, target_frame和虚拟大地图map的关系



https://blog.csdn.net/Leo_Xj/article/details/88819838

ROS与navigation教程-发布里程计消息



https://www.cnblogs.com/Ezekiel/p/9907812.html

cartographer参数调整











