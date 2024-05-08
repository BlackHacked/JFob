---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.cc
```cpp
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
```
1. 调用preprocess()对点云数据进行预处理。
2. 生成anchor mask。
3. 将预处理的数据复制到PFE(Point Feature Extractor)的输入buffers。
4. 使用TensorRT执行PFE推理,得到pillar特征。
5. 将pillar特征scatter到伪图像上,得到RPN网络的输入。
6. 将伪图像复制到RPN的输入buffers。
7. 使用TensorRT执行RPN推理,生成原始的检测框（分类，方向，边界框）
8. 调用后处理函数doPostprocessCuda,对RPN输出进行解码和NMS,生成最终检测框结果。
9. 释放CUDA流和中间buffers。