---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points_cuda.cu
```cpp
void PreprocessPointsCuda::doPreprocessPointsCuda(
    const float* dev_points, const int in_num_points, int* dev_x_coors,
    int* dev_y_coors, float* dev_num_points_per_pillar, float* dev_pillar_x,
    float* dev_pillar_y, float* dev_pillar_z, float* dev_pillar_i,
    float* dev_x_coors_for_sub_shaped, float* dev_y_coors_for_sub_shaped,
    float* dev_pillar_feature_mask, int* dev_sparse_pillar_map,
    int* host_pillar_count) {
//初始化dev_pillar_count_histo_、dev_counter_、dev_pillar_count_
  GPU_CHECK(cudaMemset(dev_pillar_count_histo_, 0,
                       GRID_Y_SIZE_ * GRID_X_SIZE_ * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_counter_, 0, sizeof(int)));
  GPU_CHECK(cudaMemset(dev_pillar_count_, 0, sizeof(int)));

  //DIVUP()除法向上取整
  //NUM_THREADS_:number of threads when launching cuda kernel，是block的大小，就是线程的数量
  //num_block:是grid的大小，即block的数量。总点数除运行核函数需要的线程数是需要使用的线程块数量，向上取整确保数量足够
  int num_block = DIVUP(in_num_points, NUM_THREADS_);
  //核函数会运行block*thread,即总点数次
  make_pillar_histo_kernel<<<num_block, NUM_THREADS_>>>(
      dev_points, dev_pillar_x_in_coors_, dev_pillar_y_in_coors_,
      dev_pillar_z_in_coors_, dev_pillar_i_in_coors_, dev_pillar_count_histo_,
      in_num_points, MAX_NUM_POINTS_PER_PILLAR_, GRID_X_SIZE_, GRID_Y_SIZE_,
      GRID_Z_SIZE_, MIN_X_RANGE_, MIN_Y_RANGE_, MIN_Z_RANGE_, PILLAR_X_SIZE_,
      PILLAR_Y_SIZE_, PILLAR_Z_SIZE_, NUM_BOX_CORNERS_);
//函数执行GRID_X_SIZE_(x方向格数) * GRID_Y_SIZE_(y方向格数)，总格数次
  make_pillar_index_kernel<<<GRID_X_SIZE_, GRID_Y_SIZE_>>>(
      dev_pillar_count_histo_, dev_counter_, dev_pillar_count_, dev_x_coors,
      dev_y_coors, dev_x_coors_for_sub_, dev_y_coors_for_sub_,
      dev_num_points_per_pillar, dev_sparse_pillar_map, MAX_NUM_PILLARS_,
      MAX_NUM_POINTS_PER_PILLAR_, GRID_X_SIZE_, PILLAR_X_SIZE_, PILLAR_Y_SIZE_,
      NUM_INDS_FOR_SCAN_);
//拷贝dev_pillar_count_到host_pillar_count
//enum cudaMemcpyKind
/*CUDA memory copy types
Values
cudaMemcpyHostToHost = 0
Host -> Host
cudaMemcpyHostToDevice = 1
Host -> Device
cudaMemcpyDeviceToHost = 2
Device -> Host
cudaMemcpyDeviceToDevice = 3
Device -> Device
cudaMemcpyDefault = 4
Direction of the transfer is inferred from the pointer values. Requires unified virtual addressin*/
  GPU_CHECK(cudaMemcpy(host_pillar_count, dev_pillar_count_, sizeof(int),
                       cudaMemcpyDeviceToHost));
//函数执行pillar数*每个pillar的点数，即总点数次
  make_pillar_feature_kernel<<<host_pillar_count[0], MAX_NUM_POINTS_PER_PILLAR_>>>(
      dev_pillar_x_in_coors_, dev_pillar_y_in_coors_, dev_pillar_z_in_coors_,
      dev_pillar_i_in_coors_, dev_pillar_x, dev_pillar_y, dev_pillar_z,
      dev_pillar_i, dev_x_coors, dev_y_coors, dev_num_points_per_pillar,
      MAX_NUM_POINTS_PER_PILLAR_, GRID_X_SIZE_);
//函数执行设定的最大pillar数*每个pillar的最大点数，即最大点数次
  make_extra_network_input_kernel<<<MAX_NUM_PILLARS_, MAX_NUM_POINTS_PER_PILLAR_>>>(
      dev_x_coors_for_sub_, dev_y_coors_for_sub_, dev_num_points_per_pillar,
      dev_x_coors_for_sub_shaped, dev_y_coors_for_sub_shaped,
      dev_pillar_feature_mask, MAX_NUM_POINTS_PER_PILLAR_);
}

```
input:
const float* dev_points：点云
const int in_num_points：点数
return:
int* dev_x_coors：pillar的x索引
int* dev_y_coors：pillar的y索引
float* dev_num_points_per_pillar: 每个pillar的点数
float* dev_pillar_x：pillar中每个点的x坐标
float* dev_pillar_y：pillar中每个点的y坐标
float* dev_pillar_z：pillar中每个点的z坐标
float* dev_pillar_i: pillar中每个点的强度
float* dev_x_coors_for_sub_shaped：pillar的x坐标
float* dev_y_coors_for_sub_shaped：pillar的y坐标
float* dev_pillar_feature_mask：pillar掩码，表示点是否有效,是否没有超出一个柱内点数的限量
int* dev_sparse_pillar_map：记录存在pillar的grid位置
int* host_pillar_count：pillar总数