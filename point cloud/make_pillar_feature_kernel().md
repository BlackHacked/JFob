---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points_cuda.cu
```cpp
__global__ void make_pillar_feature_kernel(
    float* dev_pillar_x_in_coors, float* dev_pillar_y_in_coors,
    float* dev_pillar_z_in_coors, float* dev_pillar_i_in_coors,
    float* dev_pillar_x, float* dev_pillar_y, float* dev_pillar_z,
    float* dev_pillar_i, int* dev_x_coors, int* dev_y_coors,
    float* dev_num_points_per_pillar, const int max_points,
    const int GRID_X_SIZE) {
//block是pilllar的数量，blockId是pillar的id
  int ith_pillar = blockIdx.x;
//声明变量，在此pillar中的点数
  int num_points_at_this_pillar = dev_num_points_per_pillar[ith_pillar];
//thread参数是pillar内的点数，threadIdx.x是点的id
  int ith_point = threadIdx.x;
//对冗余线程不做处理
  if (ith_point >= num_points_at_this_pillar) {
    return;
  }
  //pillar的x索引
  int x_ind = dev_x_coors[ith_pillar];
  //pillar的y索引
  int y_ind = dev_y_coors[ith_pillar];
  //当前点的索引：基于pillar的索引，与grid的x和y无关
  int pillar_ind = ith_pillar * max_points + ith_point;
  //点的索引：基于pillar的x和y索引
  int coors_ind =
      y_ind * GRID_X_SIZE * max_points + x_ind * max_points + ith_point;
  //点的xyzi：基于pillar的索引
  dev_pillar_x[pillar_ind] = dev_pillar_x_in_coors[coors_ind];
  dev_pillar_y[pillar_ind] = dev_pillar_y_in_coors[coors_ind];
  dev_pillar_z[pillar_ind] = dev_pillar_z_in_coors[coors_ind];
  dev_pillar_i[pillar_ind] = dev_pillar_i_in_coors[coors_ind];
}
```
输入参数
float* dev_pillar_x_in_coors, 点的x坐标，基于点的index
float* dev_pillar_y_in_coors, 点的y坐标，基于点的index
float* dev_pillar_z_in_coors, 点的z坐标，基于点的index
float* dev_pillar_i_in_coors, 点的强度，基于点的index
int* dev_x_coors, pillar的x索引
int* dev_y_coors, pillar的y索引
float* dev_num_points_per_pillar, pillar中的点数，基于pillar的index
const int max_points, MAX_NUM_POINTS_PER_PILLAR_:100
const int GRID_X_SIZE x方向上的格数
输出参数
float* dev_pillar_x, 点的x坐标，基于pillar的index
float* dev_pillar_y, 点的y坐标，基于pillar的index
float* dev_pillar_z, 点的z坐标，基于pillar的index
float* dev_pillar_i, 点的强度，基于pillar的index
