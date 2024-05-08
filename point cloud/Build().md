---
tags: function
---
/modules/perception/lidar/lib/object_builder/object_builder.cc
```cpp
bool ObjectBuilder::Build(const ObjectBuilderOptions& options, LidarFrame* frame) {
	//当Lidar frame为空，返回false
  if (frame == nullptr) {
    return false;
  }
  std::vector<ObjectPtr>* objects = &(frame->segmented_objects);
  
  for (size_t i = 0; i < objects->size(); ++i) {
    if (objects->at(i)) {
      objects->at(i)->id = static_cast<int>(i);
      ComputePolygon2D(objects->at(i));
      ComputePolygonSizeCenter(objects->at(i));
      ComputeOtherObjectInformation(objects->at(i));
    }
  }

  //filter objects with distance
  const int filter_dist_x = 50;
  const int filter_dist_y = 30;
  size_t count = 0;
  for (size_t i = 0; i < objects->size(); ++i) {
    AINFO << objects->at(i)->anchor_point[0];
    if (objects->at(i)->anchor_point[0] < filter_dist_x 
         && objects->at(i)->anchor_point[0] > -filter_dist_x
         && objects->at(i)->anchor_point[1] < filter_dist_y
         && objects->at(i)->anchor_point[1] > -filter_dist_y) {
      if (count != i) {
        objects->at(count) = objects->at(i);
      }
      ++count;
    }
  }
  objects->resize(count);

  return true;
}
```

1. 为每个检测到的object赋予id,方便追踪等后续使用
2. 计算每个object的2D多边形轮廓(ComputePolygon2D)
3. 计算每个object的多边形面积、大小、中心点等信息(ComputePolygonSizeCenter)
4. 计算每个object的一些其他信息,如速度、加速度等(ComputeOtherObjectInformation)
5. 根据anchor点位置进行距离过滤,去除距离车身过近的object. resize对象列表,移除被过滤的object