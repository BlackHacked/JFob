# 项目工程结构
linefit_ground_segmentations
├── CMakeLists.txt：用于构建ros包
├── include
│   └── ground_segmentation
│       ├── bin.h：径向分区头文件
│       ├── ground_segmentation.h：主程序头文件
│       ├── segment.h：角向分区头文件
│       ├── typedefs.h：指定数据类型别名头文件
│       └── viewer.h：可视化头文件
├── package.xml：包含元数据与依赖关系的xml文件
└── src
    ├── bin.cc：径向分区源代码
    ├── ground_segmentation.cc：主程序源代码文件
    ├── segment.cc：角向分区源代码
    └── viewer.cc：可视化源代码
    
linefit_ground_segmentations_ros
├── CMakeLists.txt：用于构建ros包 
├── launch
│   ├── segmentation.launch：启动文件
│   ├── segmentation_params.yaml：参数配置文件
├── package.xml：包含元数据与依赖关系的xml文件
├── src
│   ├── ground_segmentation_node.cc：主程序入口
│   └── ground_segmentation_test_node.cc
└── test.rviz：可视化配置文件，用于保存和加载在rviz中设置的窗口布局、插件排列和其他界面相关的配置。

# 项目构建与部署
## 开发环境设置  
1. 确保在catkin工作空间中拉取 [catkin_simple](https://github.com/catkin/catkin_simple.git) 库
```bash    
git clone https://github.com/catkin/catkin_simple.git
```
2. 确保提前安装eigen_conversions包
```bash
sudo apt install ros-noetic-eigen-conversions
```
### 编译与构建说明
1. 构建项目
 ```bash
 catkin build linefit_ground_segmentation_ros
```

## 启动方法
1. 在`linefit_ground_segmentation_ros segmentation.launch`文件中调整话题名称以及传感器高度参数： 将 `segmentation.launch` 中的 `input_topic` 参数调整为输入点云的话题名称；将 `sensor_height` 参数调整为传感器高度。
2. 通过执行以下命令启动地面分割 ROS 节点：
```bash
roslaunch linefit_ground_segmentation_ros segmentation.launch
```
3. 在 `linefit_ground_segmentation_ros/launch/segmentation_params.yaml` 文件中调整其他相关参数。
4. 在rviz中调整可视化方式并保存至`test.rviz`文件。


# 项目参数说明
## 地面条件
- `sensor_height`: 传感器离地面的高度。
- `max_dist_to_line`: 点到线（地面）的最大垂直距离。
- `max_slope`: 线的最大斜率。
- `min_slope`: 线的最小斜率。
- `max_fit_error`: 点在线拟合中的最大误差。
- `max_start_height`: 点与估计的地面高度之间的最大高度差。
- `long_threshold`: 线的最大长度。
- `max_height`: 线点之间的最大高度差，当它们的距离大于 `long_threshold` 时。
- `line_search_angle`: 在角向上搜索线的距离。
- `gravity_aligned_frame`: 其 z 轴与重力对齐的坐标系的名称。如果指定，则输入的点云将被旋转，但不会被平移到该坐标系。如果留空，则使用传感器坐标系。
## 分割
- `r_min`: 分割开始的距离。
- `r_max`: 分割结束的距离。
- `n_bins`: 径向分区数。
- `n_segments`: 角向分段数。
## 其他
- `n_threads`: 使用的线程数。
- `latch`: 在 ROS 节点中锁定输出点云。
- `visualize`: 可视化分割结果，仅用于调试`。

# 项目工程解析