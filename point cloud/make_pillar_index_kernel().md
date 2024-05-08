---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points_cuda.cu
```cpp
__global__ void make_pillar_index_kernel(
    int* dev_pillar_count_histo, int* dev_counter, int* dev_pillar_count,
    int* dev_x_coors, int* dev_y_coors, float* dev_x_coors_for_sub,
    float* dev_y_coors_for_sub, float* dev_num_points_per_pillar,
    int* dev_sparse_pillar_map, const int max_pillars,
    const int max_points_per_pillar, const int GRID_X_SIZE,
    const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
    const int NUM_INDS_FOR_SCAN) {
  
  int x = blockIdx.x;//总block数：GRID_X_SIZE_；blockIdx.x：x方向上的第几个格
  int y = threadIdx.x;//总thread数：GRID_Y_SIZE_；threadIdx.x：y方向上的第几个格
  //从dev_pillar_count_histo数组中得到当前处理pillar中点的数量num_points_at_this_pillar
  int num_points_at_this_pillar = dev_pillar_count_histo[y * GRID_X_SIZE + x];
  //如果pillar中没有点则不做处理，返回
  if (num_points_at_this_pillar == 0) {
    return;
  }
 //dev_counter用来统计被处理的格数，每处理一个格，则+1
 //整数变量count代表当前正在处理到第几个格,作为dev_num_points_per_pillar:以count的顺序即处理顺序排序的单元格内点数；dev_x/y/z_coors:xyz方向上的pillar索引；dev_x/y_coors_for_sub:pillar在x或y方向上的坐标值
  int count = atomicAdd(dev_counter, 1);
  //当前已处理格数不多于设定的最大格数时
  if (count < max_pillars) {
  //dev_pillar_count统计格数
    atomicAdd(dev_pillar_count, 1);
    //当前格中点数多于设定数量时，舍弃多余点，将格内点数设定为最大值
    if (num_points_at_this_pillar >= max_points_per_pillar) {
      dev_num_points_per_pillar[count] = max_points_per_pillar;
    } else {
    //否则设定为从dev_pillar_count_histo中取得的当前pillar中点的数量
      dev_num_points_per_pillar[count] = num_points_at_this_pillar;
    }
    //当前处理格的x索引
    dev_x_coors[count] = x;
    //当前处理格的y索引
    dev_y_coors[count] = y;

    // TODO(...): Need to be modified after making properly trained weight
    // Will be modified in ver 1.1
    // x_offset = self.vx / 2 + pc_range[0]
    // y_offset = self.vy / 2 + pc_range[1]
    // x_sub = coors_x.unsqueeze(1) * 0.16 + x_offset
    // y_sub = coors_y.unsqueeze(1) * 0.16 + y_offset
    //pillar的x坐标
    dev_x_coors_for_sub[count] = x * PILLAR_X_SIZE + 0.1f;
    //pillar的y坐标
    dev_y_coors_for_sub[count] = y * PILLAR_Y_SIZE + -39.9f;
    //在本函数中对于没有点的格不做处理，此数组给有点存在的pillar赋值1，索引顺序是pillar的索引顺序
    dev_sparse_pillar_map[y * NUM_INDS_FOR_SCAN + x] = 1;
  }
}

```
int* dev_pillar_count_histo 每个格位的点数，格位按线程id顺序
int* dev_counter 用以统计处理到当前格为止的已处理格数
int* dev_pillar_count pillar总数量
int* dev_x_coors pillar的x索引
int* dev_y_coors pillar的y索引
float* dev_x_coors_for_sub pillar的x坐标
float* dev_y_coors_for_sub pillar的y坐标
const int max_pillars 设定好的最大pillar数
const int max_points_per_pillar 设定好的每个pillar的最大点数
const int GRID_X_SIZE 设定好的X方向上的格数
const float PILLAR_X_SIZE 单元格x方向的大小
const float PILLAR_Y_SIZE 单元格y方向的大小
const int NUM_INDS_FOR_SCAN 
输出参数：
float* dev_num_points_per_pillar：数组，每个格内的点数
int* dev_sparse_pillar_map 
输出
dev_x_coors_for_sub pillar的x坐标
dev_y_coors_for_sub pillar的y坐标