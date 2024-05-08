---
tags: function
---

```cpp
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