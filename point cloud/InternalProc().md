---
tags: function
---
```cpp
bool DetectionComponent::InternalProc(const std::shared_ptr<const drivers::PointCloud>& in_message,
                                      const std::shared_ptr<LidarFrameMessage>& out_message) {
//获取互斥锁和序列号
PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(sensor_name_);
  {
    std::unique_lock<std::mutex> lock(s_mutex_);
    s_seq_num_++;
  }
//计算点云时间戳、当前时间和时延，并发布log
  const double timestamp = in_message->measurement_time();
  const double cur_time = zhito::common::time::Clock::NowInSeconds();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Lidar:Start:msg_time[" << timestamp << sensor_name_ << ":Start:msg_time["
        << "]:cur_time[" << cur_time << "]:cur_latency[" << start_latency << "]";


//设置out_message，加入时间戳、激光雷达时间戳、序列号、prcess_stage、错误码
  out_message->timestamp_ = timestamp;
  out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
  out_message->seq_num_ = s_seq_num_;
  out_message->process_stage_ = ProcessStage::LIDAR_DETECTION;
  out_message->error_code_ = zhito::common::ErrorCode::OK;

//获取LidarFramePool中的对象frame, 将PointCloudPool中的对象cloud、时间戳和传感器信息加入到frame，把frame加入到out_message
  auto& frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->cloud = base::PointFCloudPool::Instance().Get();
  frame->timestamp = timestamp;
  frame->sensor_info = sensor_info_;
  
//Macro：开始
  PERCEPTION_PERF_BLOCK_START();

//获取激光雷达坐标系到世界坐标系的转换矩阵pose，如果获取失败则返回false
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  const double lidar_query_tf_timestamp = timestamp - lidar_query_tf_offset_ * 0.001;
  if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp, &pose)) {
    out_message->error_code_ = zhito::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get pose at time: " << lidar_query_tf_timestamp;
    return false;
  }

//Macro：结束：用于性能统计，输出执行时间的log：输出执行坐标转换的时间
//eg. 输出:  FRAME_STATISTICS:Lidar:detection:total_time[0.012345]
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name_, "detection_1::get_lidar_to_world_pose");

//把lidar坐标系到世界坐标系的转换矩阵pose加入frame
  frame->lidar2world_pose = pose;

//获取激光雷达的障碍物检测选项，设置传感器名称和lidar到imu的外参
  lidar::LidarObstacleDetectionOptions detect_opts;
  detect_opts.sensor_name = sensor_name_;
  lidar2world_trans_.GetExtrinsics(&detect_opts.sensor2novatel_extrinsics);

//调用LidarObstacleDetection的process()函数，如果执行失败则返回false
  lidar::LidarProcessResult ret = detector_->Process(detect_opts, in_message, frame.get());
  if (ret.error_code != lidar::LidarErrorCode::Succeed) {
    out_message->error_code_ = zhito::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Lidar detection process error, " << ret.log;
    return false;
  }

//Macro：结束：用于性能统计，输出执行时间的log，输出执行障碍物检测的时间
//eg. 输出:  FRAME_STATISTICS:Lidar:detection:total_time[0.012345]
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name_, "detection_2::detect_obstacle");

  return true;
}
```