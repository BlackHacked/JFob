---
tags: function
---
/modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points.cc
```cpp
void PreprocessPoints::preprocess(const float* in_points_array, int in_num_points, int* x_coors, int* y_coors, float* num_points_per_pillar,
                                  float* pillar_x, float* pillar_y, float* pillar_z, float* pillar_i, float* x_coors_for_sub_shaped,
                                  float* y_coors_for_sub_shaped, float* pillar_feature_mask, float* sparse_pillar_map,
                                  int* host_pillar_count) {
  int pillar_count = 0;
  float x_coors_for_sub[MAX_NUM_PILLARS_];
  float y_coors_for_sub[MAX_NUM_PILLARS_];
  x_coors_for_sub[0] = 0;
  y_coors_for_sub[0] = 0;
  // init variables
  //coor_to_pillaridx代表grid到pillar的映射关系，大小为grid的数量
  int coor_to_pillaridx[GRID_Y_SIZE_ * GRID_X_SIZE_];
  initializeVariables(coor_to_pillaridx, sparse_pillar_map, pillar_x, pillar_y, pillar_z, pillar_i, x_coors_for_sub_shaped,
                      y_coors_for_sub_shaped);

//NUM_BOX_CORNERS_：4
//MIN_X_RANGE_：点云数据的x方向的最小坐标值
//MIN_X_RANGE_(0.0f),
//MIN_Y_RANGE_(-39.68f),
//遍历所有点，计算每个点所在的grid的xyz
  for (int i = 0; i < in_num_points; i++) {
	  //NUM_BOX_CORNERS_：number of corners for 2D box = 4
	  //each point contains 4 values in the input array: x y z i
	  //not sure about the meaning of using that variable!
    int x_coor = std::floor((in_points_array[i * NUM_BOX_CORNERS_ + 0] - MIN_X_RANGE_) / PILLAR_X_SIZE_);
    int y_coor = std::floor((in_points_array[i * NUM_BOX_CORNERS_ + 1] - MIN_Y_RANGE_) / PILLAR_Y_SIZE_);
    int z_coor = std::floor((in_points_array[i * NUM_BOX_CORNERS_ + 2] - MIN_Z_RANGE_) / PILLAR_Z_SIZE_);
//如果grid的xyz索引超出了既定范围，则该点在不合理的范围内，跳过该点的处理
    if (x_coor < 0 || x_coor >= GRID_X_SIZE_ || y_coor < 0 || y_coor >= GRID_Y_SIZE_ || z_coor < 0 || z_coor >= GRID_Z_SIZE_) {
      continue;
    }
// reverse index
// 通过grid的索引，定义pillar索引（grid的xy索引就是pillar的索引），将二维的索引转换到一维
    int pillar_index = coor_to_pillaridx[y_coor * GRID_X_SIZE_ + x_coor];
//对于每一个pillar
    if (pillar_index == -1) {
      pillar_index = pillar_count;
      if (pillar_count >= MAX_NUM_PILLARS_) {
        break;
      }
      pillar_count += 1;
      //把pillar_index赋值回一维数组
      coor_to_pillaridx[y_coor * GRID_X_SIZE_ + x_coor] = pillar_index;
	  //对pillar的xy坐标取整
      y_coors[pillar_index] = std::floor(y_coor);
      x_coors[pillar_index] = std::floor(x_coor);

      // float y_offset = PILLAR_Y_SIZE_/ 2 + MIN_Y_RANGE_;
      // float x_offset = PILLAR_X_SIZE_/ 2 + MIN_X_RANGE_;
      // TODO(...): Need to be modified after proper training code
      // Will be modified in ver 1.1
      //取pillar的y坐标*y方向栅格的大小：是pillar相对于原点的偏移量
      y_coors_for_sub[pillar_index] = std::floor(y_coor) * PILLAR_Y_SIZE_ + -39.9f;//PILLAR_Y_SIZE_:size of y-dimention for a pillar
      x_coors_for_sub[pillar_index] = std::floor(x_coor) * PILLAR_X_SIZE_ + 0.1f;

      sparse_pillar_map[y_coor * NUM_INDS_FOR_SCAN_ + x_coor] = 1;
    }

//当pillar中点的数量不超过限定值时，将点的坐标值存到对应的pillar数组中
    int num = num_points_per_pillar[pillar_index];
    if (num < MAX_NUM_POINTS_PER_PILLAR_) {
      pillar_x[pillar_index * MAX_NUM_POINTS_PER_PILLAR_ + num] = in_points_array[i * NUM_BOX_CORNERS_ + 0];
      pillar_y[pillar_index * MAX_NUM_POINTS_PER_PILLAR_ + num] = in_points_array[i * NUM_BOX_CORNERS_ + 1];
      pillar_z[pillar_index * MAX_NUM_POINTS_PER_PILLAR_ + num] = in_points_array[i * NUM_BOX_CORNERS_ + 2];
      pillar_i[pillar_index * MAX_NUM_POINTS_PER_PILLAR_ + num] = in_points_array[i * NUM_BOX_CORNERS_ + 3];
      num_points_per_pillar[pillar_index] += 1;
    }
  }

//对于每个pillar
  for (int i = 0; i < MAX_NUM_PILLARS_; i++) {
    float x = x_coors_for_sub[i];
    float y = y_coors_for_sub[i];
    int num_points_for_a_pillar = num_points_per_pillar[i];
    for (int j = 0; j < MAX_NUM_POINTS_PER_PILLAR_; j++) {
      //将偏移量赋值给x_coors_for_sub_shaped数组
      x_coors_for_sub_shaped[i * MAX_NUM_POINTS_PER_PILLAR_ + j] = x;
      y_coors_for_sub_shaped[i * MAX_NUM_POINTS_PER_PILLAR_ + j] = y;
      //对pillar中的每个合理点赋予一个mask，合理的判定标准是点的数量是否超过在一个pillar中点数的限定数值
      //筛选掉一部分点，留下的点数是提前设定的
      if (j < num_points_for_a_pillar) {
        pillar_feature_mask[i * MAX_NUM_POINTS_PER_PILLAR_ + j] = 1.0f;
      } else {
        pillar_feature_mask[i * MAX_NUM_POINTS_PER_PILLAR_ + j] = 0.0f;
      }
    }
  }
  //将最终的pillar数量赋值host_pillar_count
  host_pillar_count[0] = pillar_count;
}
```
input:
const float* in_points_array：输入的点云数组
int in_num_points：输入点云中点的数量
return:
int* x_coors：pillar的x坐标索引(从原点开始是第几个呀)
int* y_coors：pillar的y坐标索引
float* num_points_per_pillar：每个pillar中点的数量
float* pillar_x：pillar中点的x坐标
float* pillar_y：pillar中点的y坐标
float* pillar_z：pillar中点的z坐标
float* pillar_i：pillar中点的i，intensity强度
float* x_coors_for_sub_shaped：栅格的x坐标（相对于原点偏移了多少）
float* y_coors_for_sub_shaped：栅格的y坐标（相对于原点的偏移量）
float* pillar_feature_mask：pillar的特征掩码，标记pillar中的有效点
float* sparse_pillar_map：记录存在pillar的grid位置
int* host_pillar_count：检测到的pillar总数