---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/anchor_mask_cuda.cu
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
```
int* dev_sparse_pillar_map, 
int* dev_cumsum_along_x,
int* dev_cumsum_along_y, 
const float* dev_box_anchors_min_x,
const float* dev_box_anchors_min_y, 
const float* dev_box_anchors_max_x,
const float* dev_box_anchors_max_y, 
int* dev_anchor_mask