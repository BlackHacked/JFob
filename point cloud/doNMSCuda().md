---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/nms_cuda.cu
```cpp
void NMSCuda::doNMSCuda(const int host_filter_count,
                        float *dev_sorted_box_for_nms, int *out_keep_inds,
                        int *out_num_to_keep) {
  const int col_blocks = DIVUP(host_filter_count, NUM_THREADS_);
  dim3 blocks(DIVUP(host_filter_count, NUM_THREADS_),
              DIVUP(host_filter_count, NUM_THREADS_));
  dim3 threads(NUM_THREADS_);

  uint64_t *dev_mask = NULL;
  GPU_CHECK(cudaMalloc(
      &dev_mask, host_filter_count * col_blocks * sizeof(uint64_t)));

  nms_kernel<<<blocks, threads>>>(host_filter_count, nms_overlap_threshold_,
                                  dev_sorted_box_for_nms, dev_mask,
                                  NUM_BOX_CORNERS_);

  // postprocess for nms output
  std::vector<uint64_t> host_mask(host_filter_count * col_blocks);
  GPU_CHECK(
      cudaMemcpy(&host_mask[0], dev_mask,
                 sizeof(uint64_t) * host_filter_count * col_blocks,
                 cudaMemcpyDeviceToHost));
  std::vector<uint64_t> remv(col_blocks);
  memset(&remv[0], 0, sizeof(uint64_t) * col_blocks);

  for (int i = 0; i < host_filter_count; i++) {
    int nblock = i / NUM_THREADS_;
    int inblock = i % NUM_THREADS_;

    if (!(remv[nblock] & (1ULL << inblock))) {
      out_keep_inds[(*out_num_to_keep)++] = i;
      uint64_t *p = &host_mask[0] + i * col_blocks;
      for (int j = nblock; j < col_blocks; j++) {
        remv[j] |= p[j];
      }
    }
  }
  GPU_CHECK(cudaFree(dev_mask));
}
```
- `host_filter_count`：输入参数，要进行NMS处理的过滤器数量。
- `dev_sorted_box_for_nms`：输入参数，存储了排序的用于NMS的边界框数据的设备内存的指针。
- `out_keep_inds`：输出参数，保存NMS结果保留的边界框索引的设备内存的指针。
- `out_num_to_keep`：输出参数，NMS结果中保留的边界框数量。