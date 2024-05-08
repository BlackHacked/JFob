---
tags: function
---
modules/perception/lidar/lib/detection/lidar_point_pillars/anchor_mask_cuda.cu
计算锚框掩码
```cpp
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