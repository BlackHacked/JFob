---
tags: function
---
```cpp
LidarProcessResult LidarObstacleDetection::Process(const LidarObstacleDetectionOptions& options,
                                                   const std::shared_ptr<zhito::drivers::PointCloud const>& message, LidarFrame* frame) {
//从option中获取传感器名称
//lidar_obstacle_detection.h struct LidarObstacleDetectionOptions {std::string sensor_name;Eigen::Affine3d sensor2novatel_extrinsics;};
  const auto& sensor_name = options.sensor_name;
  
//Macro: 输出log：indicator 传感器名称 sensor_name 和进入process()的当前时间戳
  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(options.sensor_name);

//Macro：开始
  PERCEPTION_PERF_BLOCK_START();

  PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics = options.sensor2novatel_extrinsics;

//Macro:结束：输出log：执行预处理的时长
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "preprocess");

//调用preprocess()做预处理
  if (cloud_preprocessor_.Preprocess(preprocessor_options, message, frame)) {
	//成功，调用ProcessCommon()
    return ProcessCommon(options, frame);
  }
  //不成功，返回LidarProcessResult：预处理失败
  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError, "Failed to preprocess point cloud.");
}
```
