```python
cmake_minimum_required(VERSION 3.0.2)
project(urban_road_filter)
add_compile_options(-std=c++17 -O2)

find_package(catkin REQUIRED COMPONENTS
dynamic_reconfigure
pcl_conversions
pcl_ros
roscpp
rospy
sensor_msgs
)

generate_dynamic_reconfigure_options(
cfg/LidarFilters.cfg
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES urban_road_filte
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(lidar_road src/lidar_segmentation.cpp src/main.cpp src/star_shaped_search.cpp
src/x_zero_method.cpp src/z_zero_method.cpp src/blind_spots.cpp)
add_dependencies(lidar_road ${PROJECT_NAME}_gencfg)
target_link_libraries(lidar_road ${catkin_LIBRARIES})
```
