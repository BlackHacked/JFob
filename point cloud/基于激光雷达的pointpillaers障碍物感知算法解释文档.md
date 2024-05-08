商用车开发院 智能网联车开发部 韩睿
# 目标检测部件
对应源代码文件：detection_component.cc
源代码路径：/modules/perception/onboard/component/detection_component.cc
```cpp
bool DetectionComponent::Init() {
//读取配置文件
//modules/perception/launcher_perception/data/lidar/models/lidar_obstacle_pipeline/lidar_front/lidar_obstacle_detection.conf
  LidarDetectionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  ADEBUG << "Lidar Component Configs: " << comp_config.DebugString();
//初始化变量
  output_channel_name_ = comp_config.output_channel_name();
  sensor_name_ = comp_config.sensor_name();
  lidar2novatel_tf2_child_frame_id_ = comp_config.lidar2novatel_tf2_child_frame_id();
  lidar_query_tf_offset_ = static_cast<float>(comp_config.lidar_query_tf_offset());
  enable_hdmap_ = comp_config.enable_hdmap();
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);
//创建 writer 节点
#ifdef PUB_LIDAR_DETECTION
  pub_writer_ = node_->CreateWriter<PerceptionObstacles>("/zhito/perception/detection");
#endif

//初始化算法插件 algorithm plugin
  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init detection component algorithm plugin.";
    return false;
  }
  return true;
}
bool DetectionComponent::InitAlgorithmPlugin() {
//读取传感器信息sensor info
  ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_, &sensor_info_));
//std::unique_ptr<lidar::LidarObstacleDetection> detector_; 定义一个lidar::LidarObstacleDetection对象的智能指针detecor_
//unique_ptr：smart pointer：- 表达对对象的唯一拥有权；自动管理对象生命周期(创建和删除)；防止对象被无意中复制
//判断detector_是否正确实例化
  detector_.reset(new lidar::LidarObstacleDetection);
  if (detector_ == nullptr) {
    AERROR << "sensor_name_ " << "Failed to get detection instance";
    return false;
  }
  
//定义初始化选项，包括传感器名称和是否使用高精地图
  lidar::LidarObstacleDetectionInitOptions init_options;
  init_options.sensor_name = sensor_name_;
  init_options.enable_hdmap_input = FLAGS_obs_enable_hdmap_input && enable_hdmap_;

//调用 lidar_obstacle_detection的Init()函数初始化检测器
  if (!detector_->Init(init_options)) {
    AINFO << "sensor_name_ "
          << "Failed to init detection.";
    return false;
  }
//调用lidar2world_trans_.Init初始化lidar到世界坐标系的转换
  lidar2world_trans_.Init(lidar2novatel_tf2_child_frame_id_);
  return true;
}
ool DetectionComponent::Proc(const std::shared_ptr<drivers::PointCloud>& message) {
//输出log，包括消息时间戳和当前时间戳
  AINFO << "\n---------------------------------------------------------------------";
  AINFO << "Enter detection component, message timestamp: " << message->measurement_time()
        << " current timestamp: " << zhito::common::time::Clock::NowInSeconds();

//创建智能指针：输出消息（检测结果）和发布消息
  std::shared_ptr<LidarFrameMessage> out_message(new (std::nothrow) LidarFrameMessage);
  std::shared_ptr<PerceptionObstacles> pub_message(new (std::nothrow) PerceptionObstacles);

//调用Internalproc()函数
  bool status = InternalProc(message, out_message);

//发送消息和返回值
//如果Internalproc()执行成功，则发送out_message消息
  if (status) {
    writer_->Write(out_message);
    //输出log并打印障碍物数量
    AINFO << "Send lidar detect output message. " << out_message->lidar_frame_->segmented_objects.size();
    for (int i = 0; i < out_message->lidar_frame_->segmented_objects.size(); i++)
    //输出每个障碍物的中心信息
      AINFO << out_message->lidar_frame_->segmented_objects[i]->center;
  }

//如果tranformMessage()执行成功，则发送pub_message消息 
#ifdef PUB_LIDAR_DETECTION
  bool trans_succ = TransformMessage(out_message, pub_message);
  if (trans_succ) {
    pub_writer_->Write(pub_message);
  }
#endif

//返回Internalproc()的执行结果
  return status;
}
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
bool DetectionComponent::TransformMessage(const std::shared_ptr<const LidarFrameMessage>& out_message,
                                          const std::shared_ptr<PerceptionObstacles>& pub_message) {
//pub_message的类型是PerceptionObstacles，obstacle作为pub_message的指针，可以修改pub_message的内容
//header作为pub_message头部信息的指针，可修改pub_message的头部信息
  PerceptionObstacles* obstacles = pub_message.get();
  zhito::common::Header* header = obstacles->mutable_header();
//从out_message中读取时间戳、激光雷达时间戳、序列号等信息加入到pub_message的头部
  header->set_timestamp_sec(out_message->timestamp_);
  header->set_lidar_timestamp(out_message->lidar_timestamp_);
  header->set_sequence_num(out_message->seq_num_);
  header->set_module_name("perception_segmentation");
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);
//从out_message中读取错误码
  obstacles->set_error_code(out_message->error_code_);
//读取障碍物信息
//遍历out_message中的障碍物列表object，调用MsgSerializer::ConvertObjectToPb()把障碍物obj转换为PerceptionObstacles，并设置id
  const std::vector<base::ObjectPtr>& objects = out_message->lidar_frame_->segmented_objects;
  int id_count = 0;
  for (const auto& obj : objects) {
    PerceptionObstacle* obstacle = obstacles->add_perception_obstacle();
//返回转换结果
    if (!MsgSerializer::ConvertObjectToPb(obj, obstacle)) {
      AERROR << "segementation component ConvertObjectToPb failed, Object:" << obj->ToString();
      return false;
    }
    obstacle->set_id(id_count++);
    // std::cout<<"out_message_id:"<<obj->id<<"/n"
    //          <<"pub_message_id:"<<obstacle->id()<<std::endl;
  }
  // std::cout<<"out_message_size: "<<objects.size()<<"\n"
  //          <<"pub_message_size:
  //          "<<obstacles->perception_obstacle().size()<<std::endl;
  return true;
}
```

# pointpillars算法实现

## 前处理
对应源代码文件：pointcloud_preprocessor.cc
源代码路径：/modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.cc

```cpp
bool PointCloudPreprocessor::Preprocess(const PointCloudPreprocessorOptions& options,
                                        const std::shared_ptr<zhito::drivers::PointCloud const>& message, LidarFrame* frame) const {

//检查frame、frame->cloud、frame->world_cloud是否为空指针
  if (frame == nullptr) {
    return false;
  }
  if (frame->cloud == nullptr) {
     //获取单例，初始化
    frame->cloud = base::PointFCloudPool::Instance().Get();
  }
  if (frame->world_cloud == nullptr) {
    frame->world_cloud = base::PointDCloudPool::Instance().Get();
  }

  // for sweeper
  //基于chassis_angle_定义变换矩阵chassis_own_transform_M
  Eigen::Matrix4d chassis_own_transform_M = Eigen::Matrix4d::Identity();
  chassis_own_transform_M << cos(chassis_angle_), sin(chassis_angle_), 0, 0,
                       -sin(chassis_angle_), cos(chassis_angle_), 0, 0,
                       0, 0, 1, 0,
                       0, 0, 0, 1;
//创建并初始化仿射变换chassis_own_transform
  Eigen::Affine3d chassis_own_transform = Eigen::Affine3d::Identity();
  chassis_own_transform.matrix() = chassis_own_transform_M;
  // Eigen::Matrix3d rotation_45degrees = Eigen::Matrix3d::Identity();
  // rotation_45degrees << cos(45.0/180.0 * M_PI), sin(45.0/180.0 * M_PI), 0,
  //                       -sin(45.0/180.0 * M_PI), cos(45.0/180.0 * M_PI), 0,
  //                       0, 0, 1;

//设置frame->cloud的时间戳为message的时间戳
  frame->cloud->set_timestamp(message->measurement_time());

//message中存在点
  if (message->point_size() > 0) {
	//将message的point_size()点数量值赋给frame(指向LidarFrame的指针）中点云的预留空间
	//在frame中预留与message中点云数量相同的空间，以便存储点云数据，减少动态内存分配的开销。
    frame->cloud->reserve(message->point_size());
        
    //遍历message中的点（指向LidarFrame对象的指针）
    base::PointF point;
    for (int i = 0; i < message->point_size(); ++i) {
      //创建指向PointXYZIT(T:timestamp)结构体的指针pt，赋值message中的点
      const zhito::drivers::PointXYZIT& pt = message->point(i);
      //过滤NaN或infinity
      if (filter_naninf_points_) {
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
          continue;
        }
        //fabs()计算浮点数的绝对值
        if (fabs(pt.x()) >= kPointInfThreshold || fabs(pt.y()) >= kPointInfThreshold || fabs(pt.z()) >= kPointInfThreshold) {
          continue;
        }
      }

      //创建三维向量vec3d_lidar，将pt的三个分量赋值给vec3d_lidar，将vec3d_lidar的值赋给vec3d_novatel
      Eigen::Vector3d vec3d_lidar(pt.x(), pt.y(), pt.z());
      Eigen::Vector3d vec3d_novatel = vec3d_lidar;
      /**
      //过滤box附近点
      if (filter_nearby_box_points_ && vec3d_novatel[0] < box_forward_x_ && vec3d_novatel[0] > box_backward_x_ &&
          vec3d_novatel[1] < box_forward_y_ && vec3d_novatel[1] > box_backward_y_) {
        continue;
      }
	  //过滤指定高度阈值的点
      if (filter_high_z_points_ && (pt.z() > z_threshold_ || pt.z() < z_down_threshold_)) {
        continue;
      }
      //
      if (filter_region_ && (vec3d_novatel[0] > range_forward_x_ || vec3d_novatel[0] < range_backward_x_ ||
                            vec3d_novatel[1] > range_forward_y_ || vec3d_novatel[1] < range_backward_y_)) {
        continue;
      }
      // j7
      if (j7_label_ && vec3d_novatel[0] > trunk_left_edge_xmin_ && vec3d_novatel[0] < trunk_left_edge_xmax_ &&
           vec3d_novatel[1] > trunk_left_edge_ymin_ && vec3d_novatel[1] < trunk_left_edge_ymax_) {
          continue;
      }
      if (j7_label_ && vec3d_novatel[0] > trunk_right_edge_xmin_ && vec3d_novatel[0] < trunk_right_edge_xmax_ &&
           vec3d_novatel[1] > trunk_right_edge_ymin_ && vec3d_novatel[1] < trunk_right_edge_ymax_) {
          continue;
      }
      if (j7_label_ && vec3d_novatel[0] > trunk_bottom_xmin_ && vec3d_novatel[0] < trunk_bottom_xmax_ &&
            vec3d_novatel[1] > trunk_bottom_ymin_ && vec3d_novatel[1] < trunk_bottom_ymax_ &&
            vec3d_novatel[2] > trunk_bottom_zmin_ && vec3d_novatel[2] < trunk_bottom_zmax_) {
          continue;
      }
	//过滤扫地机刷子
      if (filter_sweeper_brush_ && (vec3d_novatel[0] < box_sweeper_brush_forward_x_ && vec3d_novatel[0] > box_sweeper_brush_backward_x_ &&
                            vec3d_novatel[1] < box_sweeper_brush_forward_y_ && vec3d_novatel[1] > box_sweeper_brush_backward_y_ && 
                            vec3d_novatel[2] < box_sweeper_brush_forward_z_ && vec3d_novatel[2] > box_sweeper_brush_backward_z_))
      {
        continue;
      }

      // for sweeper
      //过滤通过车辆底盘的点
      if (filter_through_chassis_ && (vec3d_novatel[0] < rough_box_forward_x_ && vec3d_novatel[0] > rough_box_backward_x_ &&
                            vec3d_novatel[1] < rough_box_forward_y_ && vec3d_novatel[1] > rough_box_backward_y_))
      {
        Eigen::Vector3d trans_point = chassis_own_transform * vec3d_novatel;
        if (filter_nearby_box_points_ && trans_point[0] < box_forward_x_ && trans_point[0] > box_backward_x_ &&
                                         trans_point[1] < box_forward_y_ && trans_point[1] > box_backward_y_) {
          continue;
        }
      }
      // filter point cloud for CNNSeg. <cnn_segmentation.cc 330 line>
      // Eigen::MatrixXd mat3d_lidar(3, 1);
      // mat3d_lidar(0, 0) = pt.x();
      // mat3d_lidar(1, 0) = pt.y();
      // mat3d_lidar(2, 0) = pt.z();
      // Eigen::MatrixXd vec3d_roration(3, 1);
      // vec3d_roration = rotation_45degrees * mat3d_lidar;
      // if (vec3d_roration(0, 0) <= -kPointInfThreshold || vec3d_roration(0, 0) >= kPointInfThreshold || vec3d_roration(1, 0) <= -kPointInfThreshold || vec3d_roration(1, 0) >= kPointInfThreshold)
      // {
      //   continue;
      // }

	//将过滤后的数据点添加到frame->cloud
      point.x = pt.x();
      point.y = pt.y();
      point.z = pt.z();
      //static_cast<>值的类型转换
      point.intensity = static_cast<float>(pt.intensity());
      //push_back()：cloud对象的成员函数，用于在末尾添加一个数据点
      //inline void push_back(const PointT& point, double timestamp,float height = std::numeric_limits<float>::max(),int32_t beam_id = -1, uint8_t label = 0)
      //FLT_MAX是float类型的最大值
      //1e-9: 纳秒到秒
      frame->cloud->push_back(point, static_cast<double>(pt.timestamp()) * 1e-9, FLT_MAX, i, 0);
    }
    */
    
    const double region_mid_time1 = zhito::common::time::Clock::NowInSeconds();//time1
    //执行RegionFilter()方法
    RegionFilter(message, frame);
    const double region_mid_time2 = zhito::common::time::Clock::NowInSeconds();//time1
    
    //PclOutlierFilter(frame);

    trailerDetection(frame);

    #if POINTCLOUD_PREPROCESSOR_VISUALIZATION
      publishTrailerMessage();
    #endif
    //获取当前时间
    const double region_mid_time3 = zhito::common::time::Clock::NowInSeconds();//time1
    //执行TransformCloud()方法
    TransformCloud(frame->cloud, frame->lidar2world_pose, frame->world_cloud);
    //获取当前时间
    const double region_mid_time4 = zhito::common::time::Clock::NowInSeconds();//time1

	//获取时间间隔，输出log
    const double time_cost_1 = (region_mid_time2 - region_mid_time1) * 1e3;
    const double time_cost_2 = (region_mid_time3 - region_mid_time2) * 1e3;
    const double time_cost_3 = (region_mid_time4 - region_mid_time3) * 1e3;
    const double time_cost_4 = (region_mid_time4 - region_mid_time1) * 1e3;
    AINFO << "Preprocess time;     Region filter and Downsample: " << time_cost_1 << "   Transform: " << time_cost_3 << "   Total:" << time_cost_4;
  }
  return true;
}
/modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.cc
```cpp
bool PointCloudPreprocessor::RegionFilter(const std::shared_ptr<zhito::drivers::PointCloud const>& message, LidarFrame* frame) const {

//声明pcl格式的点云对象
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_voxel(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_condition_range(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_condition_truck(new pcl::PointCloud<pcl::PointXYZI>);

  const double region_filter_mid_time1 = zhito::common::time::Clock::NowInSeconds();//time1
  
  // cloud_original->points.resize(message->point_size());
  // for (int i = 0; i < message->point_size(); ++i) {
  //   const zhito::drivers::PointXYZIT& pt = message->point(i);
  //   pcl::PointXYZI point;
  //   point.x = pt.x();
  //   point.y = pt.y();
  //   point.z = pt.z();
  //   point.intensity = pt.intensity();
  //   cloud_original->push_back(point);
  // }

//将原始点云的width和height属性赋值给cloud_original
  cloud_original->width = message->width();
  cloud_original->height = message->height();
  cloud_original->is_dense = false;

//在PCL中,一个点云是由width, height和points属性来表示的:width:点云的宽度,即一个扫描线内点的个数。height:点云的高度,即扫描线的个数。points:点云中的点列表,大小为width * height。
//当点云数据中weight*height的值和size不匹配时，重置weight为1，height为点云的size
  if (cloud_original->width * cloud_original->height != static_cast<unsigned int>(message->point_size())) {
    cloud_original->width = 1;
    cloud_original->height = message->point_size();
  }
  cloud_original->points.resize(message->point_size());

//把message中的点转换为pcl格式
  for (size_t i = 0; i < cloud_original->points.size(); ++i) {
    const zhito::drivers::PointXYZIT& pt = message->point(i);
    cloud_original->points[i].x = pt.x();
    cloud_original->points[i].y = pt.y();
    cloud_original->points[i].z = pt.z();
    cloud_original->points[i].intensity = pt.intensity();
  }

  const double region_filter_mid_time2 = zhito::common::time::Clock::NowInSeconds();//time1

//噪声滤波
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud_original);
  vg.setLeafSize((float)0.1f, (float)0.1f, (float)0.1f);
  vg.filter(*cloud_after_voxel);
  AINFO << "cloud_after_voxelgrid size: " << cloud_original->points.size() << "  " << cloud_after_voxel->points.size();
  const double region_filter_mid_time3 = zhito::common::time::Clock::NowInSeconds();//time1

//条件滤波，设置点云xyz范围
//ConditionAnd 
  pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());

  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, range_backward_x_)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, range_forward_x_)));

  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, range_backward_y_)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, range_forward_y_)));

  // z值 大于-5.0
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, -3.0)));
  // z值 小于2.0
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, z_threshold_)));

//对于以条件分割的区域做去除处理
  pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud_after_voxel);
  condrem.setKeepOrganized(false);
  condrem.filter(*cloud_after_condition_range);
  
//条件滤波：ConditionOr
  pcl::ConditionOr<pcl::PointXYZI>::Ptr truck_range_cond(new pcl::ConditionOr<pcl::PointXYZI>());

  truck_range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, box_forward_x_)));
  truck_range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, box_backward_x_)));

  truck_range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, box_forward_y_)));
  truck_range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, box_backward_x_)));

//去除自车点云
  pcl::ConditionalRemoval<pcl::PointXYZI> condrem_truck;
  condrem_truck.setCondition(truck_range_cond);
  condrem_truck.setInputCloud(cloud_after_condition_range);
  condrem_truck.setKeepOrganized(false);
  condrem_truck.filter(*cloud_after_condition_truck);

//输出log：过滤后点云中点的数量（size）
  AINFO << "cloud_after_condition size: " << cloud_after_condition_range->points.size() << "  " << cloud_after_condition_truck->points.size();
  const double region_filter_mid_time4 = zhito::common::time::Clock::NowInSeconds();//time1


  frame->cloud->clear();
  frame->cloud->reserve(cloud_after_condition_truck->size());
  for (unsigned int i = 0; i < cloud_after_condition_truck->points.size(); i++)
  {
    base::PointF point;
    pcl::PointXYZI pt = cloud_after_condition_truck->points[i];
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.intensity = static_cast<float>(pt.intensity);
    frame->cloud->push_back(point, 0.0,
                            FLT_MAX, i, 0);
  }
  const double region_filter_mid_time5 = zhito::common::time::Clock::NowInSeconds();//time1

  // const double time_cost_1 = (region_filter_mid_time2 - region_filter_mid_time1) * 1e3;
  // const double time_cost_2 = (region_filter_mid_time3 - region_filter_mid_time2) * 1e3;
  // const double time_cost_3 = (region_filter_mid_time4 - region_filter_mid_time3) * 1e3;
  // const double time_cost_4 = (region_filter_mid_time5 - region_filter_mid_time4) * 1e3;
  //AINFO << "Region filter process cost time:" << time_cost_1 << "  " << time_cost_2 << "  " << time_cost_3 << "  " << time_cost_4;

  return true;
}

bool PointCloudPreprocessor::RegionFilter(const std::shared_ptr<zhito::drivers::PointCloud const>& message, LidarFrame* frame) const {

//声明pcl格式的点云对象
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_voxel(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_condition_range(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_condition_truck(new pcl::PointCloud<pcl::PointXYZI>);

  const double region_filter_mid_time1 = zhito::common::time::Clock::NowInSeconds();//time1
  
  // cloud_original->points.resize(message->point_size());
  // for (int i = 0; i < message->point_size(); ++i) {
  //   const zhito::drivers::PointXYZIT& pt = message->point(i);
  //   pcl::PointXYZI point;
  //   point.x = pt.x();
  //   point.y = pt.y();
  //   point.z = pt.z();
  //   point.intensity = pt.intensity();
  //   cloud_original->push_back(point);
  // }

//将原始点云的width和height属性赋值给cloud_original
  cloud_original->width = message->width();
  cloud_original->height = message->height();
  cloud_original->is_dense = false;

//在PCL中,一个点云是由width, height和points属性来表示的:width:点云的宽度,即一个扫描线内点的个数。height:点云的高度,即扫描线的个数。points:点云中的点列表,大小为width * height。
//当点云数据中weight*height的值和size不匹配时，重置weight为1，height为点云的size
  if (cloud_original->width * cloud_original->height != static_cast<unsigned int>(message->point_size())) {
    cloud_original->width = 1;
    cloud_original->height = message->point_size();
  }
  cloud_original->points.resize(message->point_size());

//把message中的点转换为pcl格式
  for (size_t i = 0; i < cloud_original->points.size(); ++i) {
    const zhito::drivers::PointXYZIT& pt = message->point(i);
    cloud_original->points[i].x = pt.x();
    cloud_original->points[i].y = pt.y();
    cloud_original->points[i].z = pt.z();
    cloud_original->points[i].intensity = pt.intensity();
  }

  const double region_filter_mid_time2 = zhito::common::time::Clock::NowInSeconds();//time1

//噪声滤波
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud_original);
  vg.setLeafSize((float)0.1f, (float)0.1f, (float)0.1f);
  vg.filter(*cloud_after_voxel);
  AINFO << "cloud_after_voxelgrid size: " << cloud_original->points.size() << "  " << cloud_after_voxel->points.size();
  const double region_filter_mid_time3 = zhito::common::time::Clock::NowInSeconds();//time1

//条件滤波，设置点云xyz范围
//ConditionAnd 
  pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());

  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, range_backward_x_)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, range_forward_x_)));

  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, range_backward_y_)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, range_forward_y_)));

  // z值 大于-5.0
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, -3.0)));
  // z值 小于2.0
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, z_threshold_)));

//对于以条件分割的区域做去除处理
  pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud_after_voxel);
  condrem.setKeepOrganized(false);
  condrem.filter(*cloud_after_condition_range);
  
//条件滤波：ConditionOr
  pcl::ConditionOr<pcl::PointXYZI>::Ptr truck_range_cond(new pcl::ConditionOr<pcl::PointXYZI>());

  truck_range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, box_forward_x_)));
  truck_range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, box_backward_x_)));

  truck_range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, box_forward_y_)));
  truck_range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, box_backward_x_)));

//去除自车点云
  pcl::ConditionalRemoval<pcl::PointXYZI> condrem_truck;
  condrem_truck.setCondition(truck_range_cond);
  condrem_truck.setInputCloud(cloud_after_condition_range);
  condrem_truck.setKeepOrganized(false);
  condrem_truck.filter(*cloud_after_condition_truck);

//输出log：过滤后点云中点的数量（size）
  AINFO << "cloud_after_condition size: " << cloud_after_condition_range->points.size() << "  " << cloud_after_condition_truck->points.size();
  const double region_filter_mid_time4 = zhito::common::time::Clock::NowInSeconds();//time1


  frame->cloud->clear();
  frame->cloud->reserve(cloud_after_condition_truck->size());
  for (unsigned int i = 0; i < cloud_after_condition_truck->points.size(); i++)
  {
    base::PointF point;
    pcl::PointXYZI pt = cloud_after_condition_truck->points[i];
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.intensity = static_cast<float>(pt.intensity);
    frame->cloud->push_back(point, 0.0,
                            FLT_MAX, i, 0);
  }
  const double region_filter_mid_time5 = zhito::common::time::Clock::NowInSeconds();//time1

  // const double time_cost_1 = (region_filter_mid_time2 - region_filter_mid_time1) * 1e3;
  // const double time_cost_2 = (region_filter_mid_time3 - region_filter_mid_time2) * 1e3;
  // const double time_cost_3 = (region_filter_mid_time4 - region_filter_mid_time3) * 1e3;
  // const double time_cost_4 = (region_filter_mid_time5 - region_filter_mid_time4) * 1e3;
  //AINFO << "Region filter process cost time:" << time_cost_1 << "  " << time_cost_2 << "  " << time_cost_3 << "  " << time_cost_4;

  return true;
}
void PointCloudPreprocessor::trailerDetection(LidarFrame* frame) const
{
  if (trailer_existed_) {
    Timer timer;
    frame->trailer_existed = true;
    if (trailerTheta(trailer_, frame->cloud)) {
      trailer_.first_trailer = false;
      trailer_.trailer_failure_number = 0;
      // for (int i=0; i<4; i++) {
      //   AINFO << "Frame trailer_polygon[" << i << "] = (" \
      //   <<  frame->trailer.trailer_polygon[i].x << ", " \
      //   << frame->trailer.trailer_polygon[i].y << ")";
      // }
      // trailer_.trailer_theta_ = trailer_theta_1;
      // AINFO << "1. trailer_theta is: " << trailer_.trailer_theta;
      // AINFO << "1. trailer_theta degree is: " << trailer_.trailer_theta / M_PI * 180;
    } else {
      // AINFO << "1. trailer_theta can not be extracted.";
      if (trailer_.trailer_failure_number > trailer_.max_trailer_failure_number) {
        trailer_.first_trailer = true;
        trailer_.trailer_failure_number = 0;
      } else {
        trailer_.trailer_failure_number++;
      }
      // getchar();
    }
    // no filtered
    // for (int i=0; i<4; i++) {
    //   frame->trailer.trailer_polygon[i].x = trailer_.trailer_polygon[i].x;
    //   frame->trailer.trailer_polygon[i].y = trailer_.trailer_polygon[i].y;
    // }
    // frame->trailer.trailer_theta = trailer_.trailer_theta;
    // removeTrailer(trailer_, frame->cloud);
    // no filtered

    // trailer theta filtered
    for (int i=0; i<4; i++) {
      frame->trailer.trailer_polygon[i].x = trailer_.filtered_trailer_polygon[i].x;
      frame->trailer.trailer_polygon[i].y = trailer_.filtered_trailer_polygon[i].y;
    }
    frame->trailer.trailer_theta = trailer_.filtered_trailer_theta;
    removeFilteredTrailer(trailer_, frame->cloud);
    // trailer theta filtered

    double trailer_time = timer.toc(true);
  } else {
    frame->trailer_existed = false;
  }
  // AINFO << "Trailer time is: " << trailer_time;
}
```


## 算法主体
## pointpillars障碍物检测算法入口
对应源代码文件：lidar_obstacle_detection.cc
源代码文件路径：/modules/perception/lidar/app/lidar_obstacle_detection.cc
```cpp
bool LidarObstacleDetection::Init(const LidarObstacleDetectionInitOptions& options) {

  // std::cout<<"---------------------------"<<std::endl;
//获取配置管理器config manager的单例
//通过config manager获取模型配置model manager
//从配置文件lidar_obstacle_detection.conf中读参数,获得检测器名detector_name_等信息
  auto& sensor_name = options.sensor_name;
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));

  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = cyber::common::GetAbsolutePath(config_file, sensor_name);
  config_file = cyber::common::GetAbsolutePath(config_file, "lidar_obstacle_detection.conf");

  LidarObstacleDetectionConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  detector_name_ = config.detector();

//判断是否使用地图,结合是否适用hdmap输入与use_map_manager的结果判断
  use_map_manager_ = config.use_map_manager();
  use_object_filter_bank_ = config.use_object_filter_bank();

  use_map_manager_ = use_map_manager_ && options.enable_hdmap_input;

//获取scene manager场景管理器的单例
  SceneManagerInitOptions scene_manager_init_options;
  ACHECK(SceneManager::Instance().Init(scene_manager_init_options));
//通过sensor_name初始化点云预处理
  PointCloudPreprocessorInitOptions preprocessor_init_options;
  preprocessor_init_options.sensor_name = sensor_name;
  ACHECK(cloud_preprocessor_.Init(preprocessor_init_options));
//若use_map_manager为真，则初始化地图管理器map manager；如果为假，则设置use_map_manager为false
  if (use_map_manager_) {
    MapManagerInitOptions map_manager_init_options;
    if (!map_manager_.Init(map_manager_init_options)) {
      AINFO << "Failed to init map manager.";
      use_map_manager_ = false;
    }
  }LidarProcessResult LidarObstacleDetection::Process(const LidarObstacleDetectionOptions& options,
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
LidarProcessResult LidarObstacleDetection::ProcessCommon(const LidarObstacleDetectionOptions& options, LidarFrame* frame) {
  //获取传感器名称
  const auto& sensor_name = options.sensor_name;
	
  PERCEPTION_PERF_BLOCK_START();
  //如果use_map_manager_为真，则执行地图更新
  if (use_map_manager_) {
    MapManagerOptions map_manager_options;
    if (!map_manager_.Update(map_manager_options, frame)) {
      return LidarProcessResult(LidarErrorCode::MapManagerError, "Failed to update map structure.");
    }
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "map_manager");

//创建detection_options对象，调用Detect()方法
//如果检测失败，则返回错误结果
  DetectionOptions detection_options;
  if (!detector_->Detect(detection_options, frame)) {
     return LidarProcessResult(LidarErrorCode::DetectionError, "Failed to detect.");
  }
  // lasersenet_detector_->cnnDetection(frame);

//创建ObjectBuilderOptions对象，调用Build()方法
//struct ObjectBuilderOptions {Eigen::Vector3d ref_center = Eigen::Vector3d(0, 0, 0);};
//如果目标构建失败，则返回错误结果
  ObjectBuilderOptions build_options;
  if (!builder_.Build(build_options, frame)) {
    return LidarProcessResult(LidarErrorCode::ObjectBuilderError, "Failed to build objects.");
  }

  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "detection");
//如果全部成功则返回正确结果
  return LidarProcessResult(LidarErrorCode::Succeed);
}
```



## pointpillars算法数据前处理部分
对应源文件：point_pillars.cc
源文件路径：modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.cc
```cpp
void PointPillars::initAnchors() {
  // allocate memory for anchors
 //为变量创建数组，长度是NUM_ANCHOR_，即总格数的一半
  anchors_px_ = new float[NUM_ANCHOR_];//锚框中心x坐标
  anchors_py_ = new float[NUM_ANCHOR_];//锚框中心y坐标
  anchors_pz_ = new float[NUM_ANCHOR_];//锚框中心z坐标
  anchors_dx_ = new float[NUM_ANCHOR_];//锚框在x方向上的尺寸
  anchors_dy_ = new float[NUM_ANCHOR_];//锚框在y方向上的尺寸
  anchors_dz_ = new float[NUM_ANCHOR_];//锚框在z方向上的尺寸
  anchors_ro_ = new float[NUM_ANCHOR_];//旋转角度
  box_anchors_min_x_ = new float[NUM_ANCHOR_];//anchor对应2D框的最小x坐标
  box_anchors_min_y_ = new float[NUM_ANCHOR_];//anchor对应2D框的最小y坐标
  box_anchors_max_x_ = new float[NUM_ANCHOR_];//anchor对应2D框的最大x坐标
  box_anchors_max_y_ = new float[NUM_ANCHOR_];//anchor对应2D框的最大y坐标
  // deallocate these memories in deconstructor

  generateAnchors(anchors_px_, anchors_py_, anchors_pz_, anchors_dx_, anchors_dy_, anchors_dz_, anchors_ro_);

void PointPillars::generateAnchors(float* anchors_px_, float* anchors_py_, float* anchors_pz_, float* anchors_dx_, float* anchors_dy_, float* anchors_dz_, float* anchors_ro_) {
  // zero clear
  for (int i = 0; i < NUM_ANCHOR_; i++) {
    anchors_px_[i] = 0;
    anchors_py_[i] = 0;
    anchors_pz_[i] = 0;
    anchors_dx_[i] = 0;
    anchors_dy_[i] = 0;
    anchors_dz_[i] = 0;
    anchors_ro_[i] = 0;
    box_anchors_min_x_[i] = 0;
    box_anchors_min_y_[i] = 0;
    box_anchors_max_x_[i] = 0;
    box_anchors_max_y_[i] = 0;
  }

  // 计算步长和偏移量
  float x_stride = PILLAR_X_SIZE_ * 2.0f;
  float y_stride = PILLAR_Y_SIZE_ * 2.0f;
  float x_offset = MIN_X_RANGE_ + PILLAR_X_SIZE_;
  float y_offset = MIN_Y_RANGE_ + PILLAR_Y_SIZE_;

  // 计算锚点的计数数组
  float anchor_x_count[NUM_ANCHOR_X_INDS_];//数组长度是一半x方向上的格数
  anchor_x_count[0] = 0;
  for (int i = 0; i < NUM_ANCHOR_X_INDS_; i++) {
    anchor_x_count[i] = static_cast<float>(i) * x_stride + x_offset;
  }
  float anchor_y_count[NUM_ANCHOR_Y_INDS_];
  anchor_y_count[0] = 0;
  for (int i = 0; i < NUM_ANCHOR_Y_INDS_; i++) {
    anchor_y_count[i] = static_cast<float>(i) * y_stride + y_offset;
  }

  float anchor_r_count[NUM_ANCHOR_R_INDS_];
  anchor_r_count[0] = 0;
  anchor_r_count[1] = M_PI / 2;

  // 生成锚点数组
  for (int y = 0; y < NUM_ANCHOR_Y_INDS_; y++) {
    for (int x = 0; x < NUM_ANCHOR_X_INDS_; x++) {
      for (int r = 0; r < NUM_ANCHOR_R_INDS_; r++) {
        int ind = y * NUM_ANCHOR_X_INDS_ * NUM_ANCHOR_R_INDS_ + x * NUM_ANCHOR_R_INDS_ + r;
        // 计算锚点的属性值并存储到对应的数组中
        anchors_px_[ind] = anchor_x_count[x];
        anchors_py_[ind] = anchor_y_count[y];
        anchors_ro_[ind] = anchor_r_count[r];
        anchors_pz_[ind] = -1 * SENSOR_HEIGHT_;
        anchors_dx_[ind] = ANCHOR_DX_SIZE_;
        anchors_dy_[ind] = ANCHOR_DY_SIZE_;
        anchors_dz_[ind] = ANCHOR_DZ_SIZE_;
      }
    }
  }
}

void PointPillars::convertAnchors2BoxAnchors(float* anchors_px, float* anchors_py, float* anchors_dx, float* anchors_dy, float* box_anchors_min_x_, float* box_anchors_min_y_, float* box_anchors_max_x_,float* box_anchors_max_y_) {
  // flipping box's dimension
  //定义数组，长度是一半总格数
  float flipped_anchors_dx[NUM_ANCHOR_];
  flipped_anchors_dx[0] = 0;
  //定义数组，宽度是一半总格数
  float flipped_anchors_dy[NUM_ANCHOR_];
  flipped_anchors_dy[0] = 0;
  //遍历锚框
  //NUM_ANCHOR_X_INDS_是x轴格数的一半 432/2
  //NUM_ANCHOR_Y_INDS_是y轴格数的一半 496/2
  for (int x = 0; x < NUM_ANCHOR_X_INDS_; x++) {
    for (int y = 0; y < NUM_ANCHOR_Y_INDS_; y++) {
      int base_ind = x * NUM_ANCHOR_Y_INDS_ * NUM_ANCHOR_R_INDS_ + y * NUM_ANCHOR_R_INDS_;
      flipped_anchors_dx[base_ind + 0] = ANCHOR_DX_SIZE_;
      flipped_anchors_dy[base_ind + 0] = ANCHOR_DY_SIZE_;
      flipped_anchors_dx[base_ind + 1] = ANCHOR_DY_SIZE_;
      flipped_anchors_dy[base_ind + 1] = ANCHOR_DX_SIZE_;
    }
  }
  for (int x = 0; x < NUM_ANCHOR_X_INDS_; x++) {
    for (int y = 0; y < NUM_ANCHOR_Y_INDS_; y++) {
      for (int r = 0; r < NUM_ANCHOR_R_INDS_; r++) {
        int ind = x * NUM_ANCHOR_Y_INDS_ * NUM_ANCHOR_R_INDS_ + y * NUM_ANCHOR_R_INDS_ + r;
        box_anchors_min_x_[ind] = anchors_px[ind] - flipped_anchors_dx[ind] / 2.0f;
        box_anchors_min_y_[ind] = anchors_py[ind] - flipped_anchors_dy[ind] / 2.0f;
        box_anchors_max_x_[ind] = anchors_px[ind] + flipped_anchors_dx[ind] / 2.0f;
        box_anchors_max_y_[ind] = anchors_py[ind] + flipped_anchors_dy[ind] / 2.0f;
      }
    }
  }
}
```cpp
void PointPillars::putAnchorsInDeviceMemory() {

GPU_CHECK(cudaMemcpy(dev_box_anchors_min_x_, box_anchors_min_x_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_box_anchors_min_y_, box_anchors_min_y_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_box_anchors_max_x_, box_anchors_max_x_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_box_anchors_max_y_, box_anchors_max_y_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

  

GPU_CHECK(cudaMemcpy(dev_anchors_px_, anchors_px_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_py_, anchors_py_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_pz_, anchors_pz_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_dx_, anchors_dx_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_dy_, anchors_dy_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_dz_, anchors_dz_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_ro_, anchors_ro_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

}

void PointPillars::preprocessCPU(const float* in_points_array, const int in_num_points) {
  //每个pillar的x坐标
  int x_coors[MAX_NUM_PILLARS_];
  x_coors[0] = 0;
  //每个pillar的y坐标
  int y_coors[MAX_NUM_PILLARS_];
  y_coors[0] = 0;
  //每个pillar中点的数量
  float num_points_per_pillar[MAX_NUM_PILLARS_];
  num_points_per_pillar[0] = 0;
  //存储pillar中每个点的x坐标
  float* pillar_x = new float[MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_];
  //存储pillar中每个点的y坐标
  float* pillar_y = new float[MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_];
  //存储pillar中每个点的z坐标
  float* pillar_z = new float[MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_];
  //存储pillar中每个点的强度
  float* pillar_i = new float[MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_];
  //存储每个grid的x坐标
  float* x_coors_for_sub_shaped = new float[MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_];
  //存储每个grid的y坐标
  float* y_coors_for_sub_shaped = new float[MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_];
  //pillar的特征掩码
  float* pillar_feature_mask = new float[MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_];
  //grid map representation for pilar-occupancy
  //存储存在pillar的grid位置
  float* sparse_pillar_map = new float[NUM_INDS_FOR_SCAN_ * NUM_INDS_FOR_SCAN_];
//调用PreprocessPoints::preprocess() 
//输入in_points_array、in_num_points，预处理后返回结果存储在 x_coors, y_coors, num_points_per_pillar, pillar_x, pillar_y, pillar_z,pillar_i, x_coors_for_sub_shaped, y_coors_for_sub_shaped, pillar_feature_mask, sparse_pillar_map数组中
  preprocess_points_ptr_->preprocess(in_points_array, in_num_points, x_coors, y_coors, num_points_per_pillar, pillar_x, pillar_y, pillar_z,
                                     pillar_i, x_coors_for_sub_shaped, y_coors_for_sub_shaped, pillar_feature_mask, sparse_pillar_map,
                                     host_pillar_count_);
//__host__​cudaError_t cudaMemcpy ( void* dst, const void* src, size_t count, cudaMemcpyKind kind )
//Copies data between host and device.
//初始化GPU上的内存空间为零
  GPU_CHECK(cudaMemset(dev_x_coors_, 0, MAX_NUM_PILLARS_ * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_y_coors_, 0, MAX_NUM_PILLARS_ * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_pillar_x_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_y_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_z_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_i_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_x_coors_for_sub_shaped_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_y_coors_for_sub_shaped_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0, MAX_NUM_PILLARS_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0, NUM_INDS_FOR_SCAN_ * NUM_INDS_FOR_SCAN_ * sizeof(int)));
//将cpu(host)上的预处理结果传输到GPU(device)
  GPU_CHECK(cudaMemcpy(dev_x_coors_, x_coors, MAX_NUM_PILLARS_ * sizeof(int), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_y_coors_, y_coors, MAX_NUM_PILLARS_ * sizeof(int), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_pillar_x_, pillar_x, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_pillar_y_, pillar_y, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_pillar_z_, pillar_z, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_pillar_i_, pillar_i, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_x_coors_for_sub_shaped_, x_coors_for_sub_shaped, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_y_coors_for_sub_shaped_, y_coors_for_sub_shaped, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_num_points_per_pillar_, num_points_per_pillar, MAX_NUM_PILLARS_ * sizeof(float), cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_pillar_feature_mask_, pillar_feature_mask, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                       cudaMemcpyHostToDevice));
  GPU_CHECK(cudaMemcpy(dev_sparse_pillar_map_, sparse_pillar_map, NUM_INDS_FOR_SCAN_ * NUM_INDS_FOR_SCAN_ * sizeof(float),
                       cudaMemcpyHostToDevice));
//释放cpu内存空间
  delete[] pillar_x;
  delete[] pillar_y;
  delete[] pillar_z;
  delete[] pillar_i;
  delete[] x_coors_for_sub_shaped;
  delete[] y_coors_for_sub_shaped;
  delete[] pillar_feature_mask;
  delete[] sparse_pillar_map;
}
void PointPillars::doInference(const float* in_points_array, const int in_num_points, std::vector<float>* out_detections) {
  preprocess(in_points_array, in_num_points);//点云预处理

//调用doAnchorMaskCuda()方法生成锚框掩码
  anchor_mask_cuda_ptr_->doAnchorMaskCuda(dev_sparse_pillar_map_, dev_cumsum_along_x_, dev_cumsum_along_y_, dev_box_anchors_min_x_,
                                          dev_box_anchors_min_y_, dev_box_anchors_max_x_, dev_box_anchors_max_y_, dev_anchor_mask_);
//创建CUDA流
  cudaStream_t stream;
  GPU_CHECK(cudaStreamCreate(&stream));

// 采用异步复制cudaMemcpyAsync将预处理结果dev_pillar_x_（device pillar x）拷贝到pfe_buffers_(point feature extractor buffers)。
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[0], dev_pillar_x_, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[1], dev_pillar_y_, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[2], dev_pillar_z_, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[3], dev_pillar_i_, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[4], dev_num_points_per_pillar_, MAX_NUM_PILLARS_ * sizeof(float), cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[5], dev_x_coors_for_sub_shaped_, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[6], dev_y_coors_for_sub_shaped_, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[7], dev_pillar_feature_mask_, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));

//将pfe_buffers_(point feature extractor buffers)添加到队列中计算[[TersorRT C++ API]]
  pfe_context_->enqueue(BATCH_SIZE_, pfe_buffers_, stream, nullptr);

//将dev_scattered_feature_这块GPU(设备端)内存使用cudaMemset设置为0,长度为RPN_INPUT_SIZE_ * sizeof(float)字节
  GPU_CHECK(cudaMemset(dev_scattered_feature_, 0, RPN_INPUT_SIZE_ * sizeof(float)));
//调用doScatterCuda函数，生成伪图像
  scatter_cuda_ptr_->doScatterCuda(host_pillar_count_[0], dev_x_coors_, dev_y_coors_, reinterpret_cast<float*>(pfe_buffers_[8]),
                                   dev_scattered_feature_);
//将dev_scattered_feature_(伪图像)复制到RPN缓存rpn_buffers_[0],利用CUDA流(stream)实现异步复制
  GPU_CHECK(cudaMemcpyAsync(rpn_buffers_[0], dev_scattered_feature_, BATCH_SIZE_ * RPN_INPUT_SIZE_ * sizeof(float),
                            cudaMemcpyDeviceToDevice, stream));
//将rpn_buffers_添加到CUDA流中计算
  rpn_context_->enqueue(BATCH_SIZE_, rpn_buffers_, stream, nullptr);
//使用cudaMemset重置dev_filter_count_为0
  GPU_CHECK(cudaMemset(dev_filter_count_, 0, sizeof(int)));
// 调用doPostprocessCuda函数对RPN输出进行解码和NMS,得到最终检测框
  postprocess_cuda_ptr_->doPostprocessCuda(
      reinterpret_cast<float*>(rpn_buffers_[1]), reinterpret_cast<float*>(rpn_buffers_[2]), reinterpret_cast<float*>(rpn_buffers_[3]),
      dev_anchor_mask_, dev_anchors_px_, dev_anchors_py_, dev_anchors_pz_, dev_anchors_dx_, dev_anchors_dy_, dev_anchors_dz_,
      dev_anchors_ro_, dev_filtered_box_, dev_filtered_score_, dev_filtered_dir_, dev_box_for_nms_, dev_filter_count_, out_detections);

  // release the stream and the buffers
  //释放CUDA流
  cudaStreamDestroy(stream);
}
void PointPillars::preprocess(const float* in_points_array, const int in_num_points) {
//根据reproduce_result_mode_判断适用CPU还是GPU做预处理
  if (reproduce_result_mode_) {
    preprocessCPU(in_points_array, in_num_points);
  } else {
    preprocessGPU(in_points_array, in_num_points);
  }
}
```cpp
void PointPillars::preprocessGPU(const float* in_points_array, const int in_num_points) {
  float* dev_points;
  //cudaMalloc ( void** devPtr, size_t size )
  //Allocate memory on the device.
  //给dev_points分配内存空间
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_points), in_num_points * NUM_BOX_CORNERS_ * sizeof(float)));
  //把in_points_array拷贝到dev_points
  GPU_CHECK(cudaMemcpy(dev_points, in_points_array, in_num_points * NUM_BOX_CORNERS_ * sizeof(float), cudaMemcpyHostToDevice));
  //初始化dev_pillar_count_histo_、dev_sparse_pillar_map_、dev_pillar_x_、dev_pillar_y_、dev_pillar_z_、dev_pillar_i_、dev_x_coors_、dev_y_coors_、dev_num_points_per_pillar_、dev_anchor_mask_
  GPU_CHECK(cudaMemset(dev_pillar_count_histo_, 0, GRID_Y_SIZE_ * GRID_X_SIZE_ * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0, NUM_INDS_FOR_SCAN_ * NUM_INDS_FOR_SCAN_ * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_pillar_x_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_y_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_z_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_i_, 0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_x_coors_, 0, MAX_NUM_PILLARS_ * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_y_coors_, 0, MAX_NUM_PILLARS_ * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0, MAX_NUM_PILLARS_ * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_anchor_mask_, 0, NUM_ANCHOR_ * sizeof(int)));

  preprocess_points_cuda_ptr_->doPreprocessPointsCuda(dev_points, in_num_points, dev_x_coors_, dev_y_coors_, dev_num_points_per_pillar_,
                                                      dev_pillar_x_, dev_pillar_y_, dev_pillar_z_, dev_pillar_i_,
                                                      dev_x_coors_for_sub_shaped_, dev_y_coors_for_sub_shaped_, dev_pillar_feature_mask_,
                                                      dev_sparse_pillar_map_, host_pillar_count_);
  //cudaFree ( void* devPtr )
  //Frees memory on the device.
  GPU_CHECK(cudaFree(dev_points));
  
}

```

## pointpillars算法检测部分
对应源文件：point_pillars_detection.cc
源文件路径：/modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars_detection.cc
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

void PointPillarsDetection::PclToArray(const base::PointFCloudPtr& pc_ptr, float* out_points_array, const float normalizing_factor) {
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto& point = pc_ptr->at(i);
    out_points_array[i * 4 + 0] = point.x;
    out_points_array[i * 4 + 1] = point.y;
    out_points_array[i * 4 + 2] = point.z;
    out_points_array[i * 4 + 3] = static_cast<float>(point.intensity / normalizing_factor);
  }
}
void PointPillarsDetection::GetObjects(std::vector<std::shared_ptr<Object>>* objects, 
									   const Eigen::Affine3d& pose,
                                       std::vector<float>* detections//原始检测框
                                       ) {
  Timer timer;
  //获取对象池
  int num_objects = detections->size() / kOutputNumBoxFeature;

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

//遍历检测框
  for (int i = 0; i < num_objects; ++i) {
    auto& object = objects->at(i);
    object->id = i;

    // read params of bounding box
    //读取框的参数
    float x = detections->at(i * kOutputNumBoxFeature + 0);
    float y = detections->at(i * kOutputNumBoxFeature + 1);
    float z = detections->at(i * kOutputNumBoxFeature + 2);
    float dx = detections->at(i * kOutputNumBoxFeature + 4);
    float dy = detections->at(i * kOutputNumBoxFeature + 3);
    float dz = detections->at(i * kOutputNumBoxFeature + 5);
    float yaw = detections->at(i * kOutputNumBoxFeature + 6);
    yaw += M_PI / 2;
    yaw = std::atan2(sinf(yaw), cosf(yaw));
    yaw = -yaw;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;

    // compute vertexes of bounding box and transform to world coordinate
    float dx2cos = dx * cosf(yaw) / 2;
    float dy2sin = dy * sinf(yaw) / 2;
    float dx2sin = dx * sinf(yaw) / 2;
    float dy2cos = dy * cosf(yaw) / 2;
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    for (int j = 0; j < 2; ++j) {
      PointF point0, point1, point2, point3;
      float vz = z + (j == 0 ? 0 : dz);
      point0.x = x + dx2cos + dy2sin;
      point0.y = y + dx2sin - dy2cos;
      point0.z = vz;
      point1.x = x + dx2cos - dy2sin;
      point1.y = y + dx2sin + dy2cos;
      point1.z = vz;
      point2.x = x - dx2cos - dy2sin;
      point2.y = y - dx2sin + dy2cos;
      point2.z = vz;
      point3.x = x - dx2cos + dy2sin;
      point3.y = y - dx2sin - dy2cos;
      point3.z = vz;
      object->lidar_supplement.cloud.push_back(point0);
      object->lidar_supplement.cloud.push_back(point1);
      object->lidar_supplement.cloud.push_back(point2);
      object->lidar_supplement.cloud.push_back(point3);
    }
    for (auto& pt : object->lidar_supplement.cloud) {
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
      trans_point = pose * trans_point;
      PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      object->lidar_supplement.cloud_world.push_back(world_point);
    }

    // classification (only detect vehicles so far)
    // TODO(chenjiahao): Fill object types completely
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    object->lidar_supplement.raw_probs.back()[static_cast<int>(base::ObjectType::VEHICLE)] = 1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(), object->lidar_supplement.raw_probs.back().end());
    object->type = static_cast<base::ObjectType>(
        std::distance(object->type_probs.begin(), std::max_element(object->type_probs.begin(), object->type_probs.end())));
  }

  collect_time_ = timer.toc(true);
}



```

## pointpillars算法数据处理部分
对应源文件：anchor_mask_cuda.cu
源文件路径：/modules/perception/lidar/lib/detection/lidar_point_pillars/anchor_mask_cuda.cu
```cpp
```cpp
void AnchorMaskCuda::doAnchorMaskCuda(
    int* dev_sparse_pillar_map, int* dev_cumsum_along_x,
    int* dev_cumsum_along_y, const float* dev_box_anchors_min_x,
    const float* dev_box_anchors_min_y, const float* dev_box_anchors_max_x,
    const float* dev_box_anchors_max_y, int* dev_anchor_mask) {
//调用scan_x()核函数
  scan_x<<<NUM_INDS_FOR_SCAN_, NUM_INDS_FOR_SCAN_ / 2,
           NUM_INDS_FOR_SCAN_ * sizeof(int)>>>(
      dev_cumsum_along_x, dev_sparse_pillar_map, NUM_INDS_FOR_SCAN_);
//调用scan_y()核函数，
  scan_y<<<NUM_INDS_FOR_SCAN_, NUM_INDS_FOR_SCAN_ / 2,
           NUM_INDS_FOR_SCAN_ * sizeof(int)>>>(
      dev_cumsum_along_y, dev_cumsum_along_x, NUM_INDS_FOR_SCAN_);
//把dev_cumsum_along_y复制到dev_sparse_pillar_map
  GPU_CHECK(cudaMemcpy(dev_sparse_pillar_map, dev_cumsum_along_y,
                       NUM_INDS_FOR_SCAN_ * NUM_INDS_FOR_SCAN_ * sizeof(int),
                       cudaMemcpyDeviceToDevice));
//调用make_anchor_mask_kernel核函数得到锚框掩码
  make_anchor_mask_kernel<<<NUM_ANCHOR_X_INDS_ * NUM_ANCHOR_R_INDS_,
                            NUM_ANCHOR_Y_INDS_>>>(
      dev_box_anchors_min_x, dev_box_anchors_min_y, dev_box_anchors_max_x,
      dev_box_anchors_max_y, dev_sparse_pillar_map, dev_anchor_mask,
      MIN_X_RANGE_, MIN_Y_RANGE_, PILLAR_X_SIZE_, PILLAR_Y_SIZE_, GRID_X_SIZE_,
      GRID_Y_SIZE_, NUM_INDS_FOR_SCAN_);
}
//参数：输出数组的指针、输入数组指针、数组的大小
__global__ void scan_x(int* g_odata, int* g_idata, int n) {
  extern __shared__ int temp[];  // allocated on invocation定义共享内存数组temp[],在调用时动态分配内存
  int thid = threadIdx.x;
  int bid = blockIdx.x;
  int bdim = blockDim.x;//block内的线程数
  int offset = 1;
//把输入数据加入到共享内存数组，每个线程加载两个元素
  temp[2 * thid] =
      g_idata[bid * bdim * 2 + 2 * thid];  // load input into shared memory
  temp[2 * thid + 1] = g_idata[bid * bdim * 2 + 2 * thid + 1];
//局部累加
  for (int d = n >> 1; d > 0; d >>= 1) {  // build sum in place up the tree
    __syncthreads();
    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      temp[bi] += temp[ai];
    }
    offset *= 2;
  }
 // clear the last element清除最后一个元素
  if (thid == 0) {
    temp[n - 1] = 0;
  }   
//全局累加
  for (int d = 1; d < n; d *= 2) {  // traverse down tree & build scan
    offset >>= 1;
    __syncthreads();
    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      int t = temp[ai];
      temp[ai] = temp[bi];
      temp[bi] += t;
    }
  }
//把累加结果写入输出数组
  __syncthreads();
  g_odata[bid * bdim * 2 + 2 * thid] =
      temp[2 * thid + 1];  // write results to device memory
  int second_ind = 2 * thid + 2;
  if (second_ind == bdim * 2) {
    g_odata[bid * bdim * 2 + 2 * thid + 1] =
        temp[2 * thid + 1] + g_idata[bid * bdim * 2 + 2 * thid + 1];
  } else {
    g_odata[bid * bdim * 2 + 2 * thid + 1] = temp[2 * thid + 2];
  }
}
__global__ void make_anchor_mask_kernel(
    const float* dev_box_anchors_min_x, const float* dev_box_anchors_min_y,
    const float* dev_box_anchors_max_x, const float* dev_box_anchors_max_y,
    int* dev_sparse_pillar_map, int* dev_anchor_mask, const float MIN_X_RANGE,
    const float MIN_Y_RANGE, const float PILLAR_X_SIZE,
    const float PILLAR_Y_SIZE, const int GRID_X_SIZE, const int GRID_Y_SIZE,
    const int NUM_INDS_FOR_SCAN) 
{
  int tid = threadIdx.x + blockIdx.x * blockDim.x;//定义标识符
  int anchor_coor[NUM_2D_BOX_CORNERS_MACRO] = {0};//用于存储锚框的坐标值
  const int GRID_X_SIZE_1 = GRID_X_SIZE - 1;  // grid_x_size - 1
  const int GRID_Y_SIZE_1 = GRID_Y_SIZE - 1;  // grid_y_size - 1

//计算锚框坐标
  anchor_coor[0] =
      floor((dev_box_anchors_min_x[tid] - MIN_X_RANGE) / PILLAR_X_SIZE);
  anchor_coor[1] =
      floor((dev_box_anchors_min_y[tid] - MIN_Y_RANGE) / PILLAR_Y_SIZE);
  anchor_coor[2] =
      floor((dev_box_anchors_max_x[tid] - MIN_X_RANGE) / PILLAR_X_SIZE);
  anchor_coor[3] =
      floor((dev_box_anchors_max_y[tid] - MIN_Y_RANGE) / PILLAR_Y_SIZE);

//限制锚框的位置
  anchor_coor[0] = max(anchor_coor[0], 0);
  anchor_coor[1] = max(anchor_coor[1], 0);
  anchor_coor[2] = min(anchor_coor[2], GRID_X_SIZE_1);
  anchor_coor[3] = min(anchor_coor[3], GRID_Y_SIZE_1);

//通过索引提取dev_sparse_pillar_map数组中右上、左下、左上以及右下角的位置的值
  int right_top = dev_sparse_pillar_map[anchor_coor[3] * NUM_INDS_FOR_SCAN +
                                        anchor_coor[2]];
  int left_bottom = dev_sparse_pillar_map[anchor_coor[1] * NUM_INDS_FOR_SCAN +
                                          anchor_coor[0]];
  int left_top = dev_sparse_pillar_map[anchor_coor[3] * NUM_INDS_FOR_SCAN +
                                       anchor_coor[0]];
  int right_bottom = dev_sparse_pillar_map[anchor_coor[1] * NUM_INDS_FOR_SCAN +
                                           anchor_coor[2]];
//通过计算锚框4个顶点值得到area
  int area = right_top - left_top - right_bottom + left_bottom;
//area可用于判断锚框的大小，如果area大于1则将dev_anchor_mask设置为1，否则设置为0
  if (area > 1) {
    dev_anchor_mask[tid] = 1;
  } else {
    dev_anchor_mask[tid] = 0;
  }
}
```

对应源文件：scatter_cuda.cu
源文件路径：/modules/perception/lidar/lib/detection/lidar_point_pillars/scatter_cuda.cu
```cpp
//pillar_count（pillar的数量）、x_coors（pillar的X索引）、y_coors（pillar的Y索引）、pfe_output（pfe的输出）和scattered_feature（散布后的特征数组）
void ScatterCuda::doScatterCuda(const int pillar_count, int *x_coors,
                                int *y_coors, float *pfe_output,
                                float *scattered_feature) {
//调用scatter_kernel函数，pillar_count是线程块(block)的数量，NUM_THREADS_是每个block中的线程数
  scatter_kernel<<<pillar_count, NUM_THREADS_>>>(
      x_coors, y_coors, pfe_output, scattered_feature, MAX_NUM_PILLARS_,
      GRID_X_SIZE_, GRID_Y_SIZE_);
}
__global__ void scatter_kernel(int *x_coors, int *y_coors, float *pfe_output,
                               float *scattered_feature,
                               const int MAX_NUM_PILLARS_,
                               const int GRID_X_SIZE, const int GRID_Y_SIZE) {
 //将线程块的索引值赋给i_pillar
  int i_pillar = blockIdx.x;
  //将当前线程在线程块中的索引赋值i_feature
  int i_feature = threadIdx.x;
  //获取柱的x坐标
  int x_ind = x_coors[i_pillar];
  //获取柱的y坐标
  int y_ind = y_coors[i_pillar];
  //获取当前特征值
  float feature = pfe_output[i_feature * MAX_NUM_PILLARS_ + i_pillar];
  //计算特征值的索引位置并将数值存储
  scattered_feature[i_feature * GRID_Y_SIZE * GRID_X_SIZE +
                    y_ind * GRID_X_SIZE + x_ind] = feature;
}
```

## 后处理
对应源代码文件：postprocess_cuda.cu
源代码路径：/modules/perception/lidar/lib/detection/lidar_point_pillars/postprocess_cuda.cu
```cpp
void PostprocessCuda::doPostprocessCuda(
    const float* rpn_box_output,  // RPN网络的边界框输出
    const float* rpn_cls_output,  // RPN网络的分类输出
    const float* rpn_dir_output,  // RPN网络的方向输出
    int* dev_anchor_mask,  // 锚点掩码
    const float* dev_anchors_px,  // 锚点的x坐标数组
    const float* dev_anchors_py,  // 锚点的y坐标数组
    const float* dev_anchors_pz,  // 锚点的z坐标数组
    const float* dev_anchors_dx,  // 锚点的宽度数组
    const float* dev_anchors_dy,  // 锚点的长度数组
    const float* dev_anchors_dz,  // 锚点的高度数组
    const float* dev_anchors_ro,  // 锚点的旋转角度数组
    float* dev_filtered_box,  // 过滤后的边界框数组
    float* dev_filtered_score,  // 过滤后的得分数组
    int* dev_filtered_dir,  // 过滤后的方向数组
    float* dev_box_for_nms,  // 用于NMS操作的边界框数组
    int* dev_filter_count,  // 过滤计数
    std::vector<float>* out_detection) {  // 输出检测结果的容器
 // 调用CUDA的过滤核函数进行边界框过滤操作（按置信度过滤）
  filter_kernel<<<NUM_ANCHOR_X_INDS_ * NUM_ANCHOR_R_INDS_,
                  NUM_ANCHOR_Y_INDS_>>>(
      rpn_box_output, rpn_cls_output, rpn_dir_output, dev_anchor_mask,
      dev_anchors_px, dev_anchors_py, dev_anchors_pz, dev_anchors_dx,
      dev_anchors_dy, dev_anchors_dz, dev_anchors_ro, dev_filtered_box,
      dev_filtered_score, dev_filtered_dir, dev_box_for_nms, dev_filter_count,
      FLOAT_MIN_, FLOAT_MAX_, score_threshold_, NUM_BOX_CORNERS_,
      NUM_OUTPUT_BOX_FEATURE_);

  int host_filter_count[1];
  GPU_CHECK(cudaMemcpy(host_filter_count, dev_filter_count, sizeof(int),
                       cudaMemcpyDeviceToHost));
  if (host_filter_count[0] == 0) {
    return;
  }

  int* dev_indexes;//dev索引
  float *dev_sorted_filtered_box, //排序后的边界框
*dev_sorted_box_for_nms;//排序后的nms边界框
  int* dev_sorted_filtered_dir;

//分配内存用于索引和边界框
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_indexes),
                       host_filter_count[0] * sizeof(int)));
  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_sorted_filtered_box),
      NUM_OUTPUT_BOX_FEATURE_ * host_filter_count[0] * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_sorted_filtered_dir),
                       host_filter_count[0] * sizeof(int)));
  GPU_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&dev_sorted_box_for_nms),
                 NUM_BOX_CORNERS_ * host_filter_count[0] * sizeof(float)));
/*
__host__ __device__ void thrust::sequence	(	const thrust::detail::execution_policy_base< DerivedPolicy > & 	exec,
ForwardIterator 	first,
ForwardIterator 	last 
)	
|exec|The execution policy to use for parallelization.|thrust::device：用于在GPU上执行
|first|The beginning of the sequence.|
|last|The end of the sequence.|

*/
//生成一个递增的序列dev_indexes
  thrust::sequence(thrust::device, dev_indexes,
                   dev_indexes + host_filter_count[0]);
//也可以根据置信度排序成序列dev_indexes
//   thrust::sort_by_key(thrust::device, dev_filtered_score,
//                       dev_filtered_score + size_t(host_filter_count[0]),
//                       dev_indexes, thrust::greater<float>());

  const int num_blocks = DIVUP(host_filter_count[0], NUM_THREADS_);
  //对过滤后的边界框和方向进行按索引排序
  sort_boxes_by_indexes_kernel<<<num_blocks, NUM_THREADS_>>>(
      dev_filtered_box, dev_filtered_dir, dev_box_for_nms, dev_indexes,
      host_filter_count[0], dev_sorted_filtered_box, dev_sorted_filtered_dir,
      dev_sorted_box_for_nms, NUM_BOX_CORNERS_, NUM_OUTPUT_BOX_FEATURE_);

  int keep_inds[host_filter_count[0]];
  keep_inds[0] = 0;
  int out_num_objects = 0;
  //对排序后的NMS边界框做非最大抑制
  nms_cuda_ptr_->doNMSCuda(host_filter_count[0], dev_sorted_box_for_nms,
                           keep_inds, &out_num_objects);

  float host_filtered_box[host_filter_count[0] * NUM_OUTPUT_BOX_FEATURE_];
  int host_filtered_dir[host_filter_count[0]];
  // 将排序后的边界框和方向从设备内存复制到主机内存
  GPU_CHECK(
      cudaMemcpy(host_filtered_box, dev_sorted_filtered_box,
                 NUM_OUTPUT_BOX_FEATURE_ * host_filter_count[0] * sizeof(float),
                 cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(host_filtered_dir, dev_sorted_filtered_dir,
                       host_filter_count[0] * sizeof(int),
                       cudaMemcpyDeviceToHost));
   // 将过滤后的边界框和方向存储到输出检测结果的容器中
  for (size_t i = 0; i < out_num_objects; i++) {
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * NUM_OUTPUT_BOX_FEATURE_ + 0]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * NUM_OUTPUT_BOX_FEATURE_ + 1]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * NUM_OUTPUT_BOX_FEATURE_ + 2]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * NUM_OUTPUT_BOX_FEATURE_ + 3]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * NUM_OUTPUT_BOX_FEATURE_ + 4]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * NUM_OUTPUT_BOX_FEATURE_ + 5]);
	//
    if (host_filtered_dir[keep_inds[i]] == 0) {
      out_detection->push_back(
          host_filtered_box[keep_inds[i] * NUM_OUTPUT_BOX_FEATURE_ + 6] + M_PI);
    } else {
      out_detection->push_back(
          host_filtered_box[keep_inds[i] * NUM_OUTPUT_BOX_FEATURE_ + 6]);
    }
  }
//释放内存
  GPU_CHECK(cudaFree(dev_indexes));
  GPU_CHECK(cudaFree(dev_sorted_filtered_box));
  GPU_CHECK(cudaFree(dev_sorted_filtered_dir));
  GPU_CHECK(cudaFree(dev_sorted_box_for_nms));
}
void PostprocessCuda::DoPostprocessCuda(
    const float* rpn_box_output, const float* rpn_cls_output,
    const float* rpn_dir_output, int* dev_anchor_mask,
    const float* dev_anchors_px, const float* dev_anchors_py,
    const float* dev_anchors_pz, const float* dev_anchors_dx,
    const float* dev_anchors_dy, const float* dev_anchors_dz,
    const float* dev_anchors_ro, float* dev_filtered_box,
    float* dev_filtered_score, int* dev_filtered_label, int* dev_filtered_dir,
    float* dev_box_for_nms, int* dev_filter_count,
    std::vector<float>* out_detection, std::vector<int>* out_label) {
  const int num_blocks_filter_kernel = DIVUP(num_anchor_, num_threads_);
  filter_kernel<<<num_blocks_filter_kernel, num_threads_>>>(
      rpn_box_output, rpn_cls_output, rpn_dir_output, dev_anchor_mask,
      dev_anchors_px, dev_anchors_py, dev_anchors_pz, dev_anchors_dx,
      dev_anchors_dy, dev_anchors_dz, dev_anchors_ro, dev_filtered_box,
      dev_filtered_score, dev_filtered_label, dev_filtered_dir, dev_box_for_nms,
      dev_filter_count, float_min_, float_max_, score_threshold_,
      num_box_corners_, num_output_box_feature_, num_class_);

  int host_filter_count[1] = {0};
  GPU_CHECK(cudaMemcpy(host_filter_count, dev_filter_count, sizeof(int),
                       cudaMemcpyDeviceToHost));
  if (host_filter_count[0] == 0) {
    return;
  }

  int* dev_indexes;
  float *dev_sorted_filtered_box, *dev_sorted_box_for_nms;
  int *dev_sorted_filtered_label, *dev_sorted_filtered_dir;
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_indexes),
                       host_filter_count[0] * sizeof(int)));
  GPU_CHECK(cudaMalloc(
      reinterpret_cast<void**>(&dev_sorted_filtered_box),
      num_output_box_feature_ * host_filter_count[0] * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_sorted_filtered_label),
                       host_filter_count[0] * sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_sorted_filtered_dir),
                       host_filter_count[0] * sizeof(int)));
  GPU_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&dev_sorted_box_for_nms),
                 num_box_corners_ * host_filter_count[0] * sizeof(float)));
 // dev_index 按顺序赋值0，1，2，3...
  thrust::sequence(thrust::device, dev_indexes,
                   dev_indexes + host_filter_count[0]);
// dev_filter_score 从大到小排列 并将索引顺序排入indexes
  thrust::sort_by_key(thrust::device, dev_filtered_score,
                      dev_filtered_score + size_t(host_filter_count[0]),
                      dev_indexes, thrust::greater<float>());

  const int num_blocks = DIVUP(host_filter_count[0], num_threads_);
  sort_boxes_by_indexes_kernel<<<num_blocks, num_threads_>>>(
      dev_filtered_box, dev_filtered_label, dev_filtered_dir, dev_box_for_nms,
      dev_indexes, host_filter_count[0], dev_sorted_filtered_box,
      dev_sorted_filtered_label, dev_sorted_filtered_dir,
      dev_sorted_box_for_nms, num_box_corners_, num_output_box_feature_);

  int keep_inds[host_filter_count[0]];
  memset(keep_inds, 0, host_filter_count[0] * sizeof(int));
  int out_num_objects = 0;
  nms_cuda_ptr_->DoNmsCuda(host_filter_count[0], dev_sorted_box_for_nms,
                           keep_inds, &out_num_objects);

  float host_filtered_box[host_filter_count[0] * num_output_box_feature_];
  int host_filtered_label[host_filter_count[0]];
  int host_filtered_dir[host_filter_count[0]];
  GPU_CHECK(
      cudaMemcpy(host_filtered_box, dev_sorted_filtered_box,
                 num_output_box_feature_ * host_filter_count[0] * sizeof(float),
                 cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(host_filtered_label, dev_sorted_filtered_label,
                       host_filter_count[0] * sizeof(int),
                       cudaMemcpyDeviceToHost));
  GPU_CHECK(cudaMemcpy(host_filtered_dir, dev_sorted_filtered_dir,
                       host_filter_count[0] * sizeof(int),
                       cudaMemcpyDeviceToHost));
  for (size_t i = 0; i < out_num_objects; ++i) {
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 0]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 1]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 2]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 3]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 4]);
    out_detection->push_back(
        host_filtered_box[keep_inds[i] * num_output_box_feature_ + 5]);

    if (host_filtered_dir[keep_inds[i]] == 0) {
      out_detection->push_back(
          host_filtered_box[keep_inds[i] * num_output_box_feature_ + 6] + M_PI);
    } else {
      out_detection->push_back(
          host_filtered_box[keep_inds[i] * num_output_box_feature_ + 6]);
    }

    out_label->push_back(host_filtered_label[keep_inds[i]]);
  }

  GPU_CHECK(cudaFree(dev_indexes));
  GPU_CHECK(cudaFree(dev_sorted_filtered_box));
  GPU_CHECK(cudaFree(dev_sorted_filtered_label));
  GPU_CHECK(cudaFree(dev_sorted_filtered_dir));
  GPU_CHECK(cudaFree(dev_sorted_box_for_nms));
}

```