 # 项目工程结构
├── cfg
		└── LidarFilters.cfg ：动态参数调整的配置文件
├── CMakeLists.txt ：用于构建ros包
├── config
		├── offroad.rviz：可视化配置文件，用于保存和加载在rviz中设置的窗口布局、插件排列和其他界面相关的配置。
		├── param-bag.yaml：参数配置文件
		└── RQT.perspective：rqt配置文件，用于保存和加载在rqt中设置的窗口布局、插件排列和其他界面相关的配置。
├── include
		└── road_filter
			└── data_structures.hpp：头文件
├── launch
		├── demo1.launch ：启动文件
├── package.xml：包含元数据与依赖关系的xml文件
└── src
	    ├── blind_spots.cpp：处理盲区的源代码文件
	    ├── lidar_segmentation.cpp：主程序源代码文件
	    ├── main.：主程序入口
	    ├── star_shaped_search.cpp：星型搜索分割源代码文件
	    ├── x_zero_method.cpp：x zero方法源代码文件
	    └── z_zero_method.cpp：z zero 方法源代码文件

# 项目构建与部署
## 开发环境设置
1. 确保开发环境中安装了ros，可以在终端中执行以下命令检查ROS是否已安装
```bash
rosversion -d
```
经验证的ros版本：ros noetic 

2. 确保提前安装PCL库

## 编译与构建说明
1. 构建项目
```bash
catkin build road_filter -DPYTHON_EXECUTABLE=/usr/bin/python3
```
>注意：ros melodic及之前的ros版本默认使用Python2，-DPYTHON_EXECUTABLE=/usr/bin/python3的目的是确保在构建过程中使用python3，而不是系统默认的Python2。如果使用“catkin build road_filter”命令构建项目可能会出现报错：  Unable to find either executable 'empy' or Python module 'em'...  try
  installing the package 'python3-empy'
2. 设置ros环境变量
```bash
source devel/setup.bash
```

## 启动方法
1. 初始化点云话题名称和帧id参数：
在 `./launch/demo1.launch` 中 调整`fixed_frame` 和`topic_name`参数
2. 在终端中执行以下命令以启动ROS主节点
```bash
roscore
```
3. 在另一个终端中，播放rosbag文件或发布sensor_msg/PointCloud2格式的点云数据
以下是播放rosbag文件的示例代码：
```bash
rosbag play xxxxx.bag
```
4. 在另一个终端中启动分割节点、可视化分割结果
```bash
roslaunch road_filter demo1.launch
```

# 项目参数说明

| 参数名称             | 参数功能说明                   | 类型（区间）/默认值     |
|----------------------|------------------------|------------------------|
| fixed_frame          | 固定帧，通常是服务订阅点云的frame id | 字符串 / 字符串           |
| topic_name           | 服务订阅点云的主题名称,          | 字符串 / 字符串           |
| x_zero_method        | 是否启用X-zero方法     | 布尔（True-False）/True |
| z_zero_method        | 是否启用Z-zero方法     | 布尔（True-False）/True |
| star_shaped_method   | 是否启用星形方法        | 布尔（True-False）/True |
| blind_spots          | 是否启用过滤盲点，通常不启用                | 布尔（True-False）/True |
| x_direction          | 过滤x方向点云               | 双向 正/负              |
| interval             | 点云的垂直分辨率        | 双精度（0-10）/0.18      |
| curb_height          | 路缘的估计最小高度（米）   | 双精度（0-10）/0.05      |
| curb_points          | 路缘上的预估点数（个）     | 整数（1-30）/5           |
| beam_zone            | 波束区域的宽度（度）       | 双精度（10-100）/30      |
| cylinder_deg_x       | 在x_zero_method中检测三角形的夹角（度） | 双精度（0-180）/150     |
| cylinder_deg_z       | 在z_zero_method中检测的两个向量的夹角（度） | 双精度（0-180）/140     |
| sector_deg           | 在star_shaped_method中的径向阈值（度）   | 双精度（0-180）/50       |
| min_x, max_x, min_y, max_y, min_z, max_z | 检测区域的大小 x, y, z（米） | 双精度（-200-200）/30    |
| dmin_param           | 检测的最小点数            | 整数（3-30）/10          |
| kdev_param           | 检测系数                | 双精度（0.5-5）/1.1225   |
| kdist_param          | 距离系数                | 双精度（0.4-10）/2       |


# 项目工程解析




















































































































































































































































































































































































