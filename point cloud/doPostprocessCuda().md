---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/postprocess_cuda.cu
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
```
```
1. 调用filter_kernel过滤低分框,只保留置信度高的框。
2. 为保留下来的框生成索引。
3. 根据索引对框进行排序。
4. 调用NMS算法进行非极大抑制。
5. 将排序并做完NMS的框复制到CPU内存。
6. 根据NMS的保留索引,从排序好的框中取出需要的框信息,形成最终检测结果。
7. 释放GPU内存。