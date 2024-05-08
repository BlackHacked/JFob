---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points_cuda.cu
```cpp
__global__ void make_extra_network_input_kernel(
    float* dev_x_coors_for_sub, float* dev_y_coors_for_sub,
    float* dev_num_points_per_pillar, float* dev_x_coors_for_sub_shaped,
    float* dev_y_coors_for_sub_shaped, float* dev_pillar_feature_mask,
    const int MAX_NUM_POINTS_PER_PILLAR) {
  //block:设定的最大pillar数 blockIdx.x当前所在的pillarid
  int ith_pillar = blockIdx.x;
  //thread: pillar的最大点数 threadIdx.x当前所在的点的id
  int ith_point = threadIdx.x;
  //x: pillar的x坐标
  float x = dev_x_coors_for_sub[ith_pillar];
  //y: pillar的y坐标
  float y = dev_y_coors_for_sub[ith_pillar];
  //当前pillar中的点数
  int num_points_for_a_pillar = dev_num_points_per_pillar[ith_pillar];
  //dev_x/y_coors_for_sub_shaped当前点所在pillar的x坐标和y坐标
  int ind = ith_pillar * MAX_NUM_POINTS_PER_PILLAR + ith_point;
  dev_x_coors_for_sub_shaped[ind] = x;
  dev_y_coors_for_sub_shaped[ind] = y;
//将当前pillar中的点数不多于设定值的pillar掩码设定为1
  if (ith_point < num_points_for_a_pillar) {
    dev_pillar_feature_mask[ind] = 1.0;
  } else {
    dev_pillar_feature_mask[ind] = 0.0;
  }
}
```
输入参数
float* dev_x_coors_for_sub, pillar的x坐标，基于pillar index
float* dev_y_coors_for_sub, pillar的y坐标，基于pillar index
float* dev_num_points_per_pillar, 每个pillar中的点数
const int MAX_NUM_POINTS_PER_PILLAR 每个pillar中的最大点数
输出参数
float* dev_x_coors_for_sub_shaped, pillar的x坐标，基于point 
float* dev_y_coors_for_sub_shaped, pillar的y坐标，基于point index
float* dev_pillar_feature_mask, pillar的特征掩码