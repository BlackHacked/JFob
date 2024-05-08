---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points_cuda.cu
```cpp
__global__ void make_pillar_histo_kernel(
    const float* dev_points, float* dev_pillar_x_in_coors,
    float* dev_pillar_y_in_coors, float* dev_pillar_z_in_coors,
    float* dev_pillar_i_in_coors, int* pillar_count_histo, const int num_points,
    const int max_points_per_pillar, const int GRID_X_SIZE,
    const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float MIN_X_RANGE,
    const float MIN_Y_RANGE, const float MIN_Z_RANGE, const float PILLAR_X_SIZE,
    const float PILLAR_Y_SIZE, const float PILLAR_Z_SIZE,
    const int NUM_BOX_CORNERS) {
//threadIdx.x(第几个thread) + blockIdx.x(第几个block) * blockDim.x(一个block里thread的数量)，即thread的全局索引(Global index)
  int th_i = threadIdx.x + blockIdx.x * blockDim.x;
  //对于冗余的线程不做处理，返回
  if (th_i >= num_points) {
    return;
  }
  //获取该点对应的pillar的xyz索引
  int y_coor = floor((dev_points[th_i * NUM_BOX_CORNERS + 1] - MIN_Y_RANGE) /
                     PILLAR_Y_SIZE);
  int x_coor = floor((dev_points[th_i * NUM_BOX_CORNERS + 0] - MIN_X_RANGE) /
                     PILLAR_X_SIZE);
  int z_coor = floor((dev_points[th_i * NUM_BOX_CORNERS + 2] - MIN_Z_RANGE) /
                     PILLAR_Z_SIZE);
//GRID_X_SIZE:number of pillars in x-coordinate
//如果该点所在的pillar在有效范围内（有效范围指提前设定过的三个方向上的最大格数GRID_X_SIZE：x方向最多格数）
  if (x_coor >= 0 && x_coor < GRID_X_SIZE && y_coor >= 0 &&
      y_coor < GRID_Y_SIZE && z_coor >= 0 && z_coor < GRID_Z_SIZE) {
    //Atomic operations are operations which are performed without interference from any other threads.
    //int atomicAdd(int* address, int val)；
    //pillar_count_histo[]数组用来统计每个格位的点数，每处理一个点，则在对应格位+1,格位按线程索引顺序排序
    //count是当前处理点所在格位的处理到目前为止的点数
    int count = atomicAdd(&pillar_count_histo[y_coor * GRID_X_SIZE + x_coor], 1);
    //当前格位中的点数不多于设定的最大值时
    if (count < max_points_per_pillar) {
	    //点的索引
      int ind = y_coor * GRID_X_SIZE * max_points_per_pillar +
                x_coor * max_points_per_pillar + count;
    //将点的xyzi值放到对应的dev_pillar_x_in_coors数组，数组的index按格位顺序排序。
      dev_pillar_x_in_coors[ind] = dev_points[th_i * NUM_BOX_CORNERS + 0];
      dev_pillar_y_in_coors[ind] = dev_points[th_i * NUM_BOX_CORNERS + 1];
      dev_pillar_z_in_coors[ind] = dev_points[th_i * NUM_BOX_CORNERS + 2];
      dev_pillar_i_in_coors[ind] = dev_points[th_i * NUM_BOX_CORNERS + 3];
    }
  }
}

```

输入参数
const float* dev_points, 
const int num_points 总点数
const int max_points_per_pillar每个柱的最大点数
const int GRID_X_SIZE x方向上的格数Number of pillars in x-coordinate
const int GRID_Y_SIZE y方向上的格数
const int GRID_Z_SIZE z方向上的格数
GRID_X_SIZE_(432),
GRID_Y_SIZE_(496),
GRID_Z_SIZE_(1),
const float MIN_X_RANGE 点云数据的x方向的最小坐标值
const float MIN_Y_RANGE 点云数据的y方向的最小坐标值
const float MIN_Z_RANGE 点云数据的z方向的最小坐标值
const float PILLAR_X_SIZE 单元格在x方向上的宽度
const float PILLAR_Y_SIZE 单元格在y方向上的宽度
const float PILLAR_Z_SIZE 单元格在z方向上的宽度
const int NUM_BOX_CORNERS 4
输出参数
float* dev_pillar_x_in_coors, 每个点的x坐标
float* dev_pillar_y_in_coors 每个点的y坐标
float* dev_pillar_z_in_coors 每个点的z坐标
float* dev_pillar_i_in_coors每个点的强度值
int* pillar_count_histo 每个格位的点数
