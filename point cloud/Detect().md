---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars_detection.cc
```cpp
bool PointPillarsDetection::Detect(const DetectionOptions& options, LidarFrame* frame) {
  // check input
  //检查frame、frame->cloud、frame->cloud->size()是否为空，若空，返回false.
  //如果没有点云帧的输入，则不进行处理
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }
  
  // record input cloud and lidar frame
  // 将原始点云帧赋值给声明的变量
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  lidar_frame_ref_ = frame;

  // check output
  //清空frame中原有的已检测物体
  frame->segmented_objects.clear();

  //声明Timer类的对象timer
  Timer timer;

//设置GPU
  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  // transform point cloud into an array
  //将点云转换为数组格式
  //定义了一个大小为原始点云size*4的动态数组points_array作为doInference函数的输入
  float* points_array = new float[original_cloud_->size() * 4];
  //调用PclToArray()函数将original_cloud_中点云的xyzi写入points_array
  PclToArray(original_cloud_, points_array, kNormalizingFactor);

  // inference
  //调用doInference()方法
  std::vector<float> out_detections;
  point_pillars_ptr_->doInference(points_array, original_cloud_->size(), &out_detections);
  //得到推理时间
  inference_time_ = timer.toc(true);

  // transfer output bounding boxs to objects
  //调用GetObjects()方法获取检测物体
  GetObjects(&frame->segmented_objects, frame->lidar2world_pose, &out_detections);

//释放数组
  delete[] points_array;
  AINFO << "PointPillars: inference: " << inference_time_ << "\t"
        << "collect: " << collect_time_;
//返回值
  return true;
}

```
1. 输入检查:确保输入的LidarFrame和点云不为空。
2. 保存原始点云,后续转换用。
3. 清空原有检测结果。
4. 设置GPU设备。
5. 将点云转换为数组格式,作为推理函数输入。
6. 调用doInference执行模型推理,得到检测框结果。
7. 将推理结果转换为Object格式的检测结果。
8. 释放临时数组。
9. 记录并打印推理时间。
10. 返回检测状态。