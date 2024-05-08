---
tags: function
---
modules/perception/lidar/lib/detection/lidar_point_pillars/postprocess_cuda.cu
```cpp
__global__ void filter_kernel(
    const float* box_preds,  // 预测的边界框数组
    const float* cls_preds,  // 预测的分类数组
    const float* dir_preds,  // 预测的方向数组
    const int* anchor_mask, //锚框掩码
    const float* dev_anchors_px,  // 锚点的x坐标数组
    const float* dev_anchors_py,  // 锚点的y坐标数组
    const float* dev_anchors_pz,  // 锚点的z坐标数组
    const float* dev_anchors_dx,  // 锚框的宽度数组
    const float* dev_anchors_dy,  // 锚框的长度数组
    const float* dev_anchors_dz,  // 锚框的高度数组
    const float* dev_anchors_ro,  // 锚点的旋转角度数组
    float* filtered_box,  // 过滤后的边界框数组
    float* filtered_score,  // 过滤后的置信度数组
    int* filtered_dir,  // 过滤后的方向数组
    float* box_for_nms,  // 用于NMS操作的边界框数组
    int* filter_count,  // 过滤计数
    const float FLOAT_MIN,  // 浮点数的最小值
    const float FLOAT_MAX,  // 浮点数的最大值
    const float score_threshold,  // 过滤的得分阈值
    const int NUM_BOX_CORNERS,  // 边界框的角点数
    const int NUM_OUTPUT_BOX_FEATURE) {  // 输出边界框的特征数
   // 获取当前线程的索引
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  
  // sigmoid函数计算置信度
  float score = 1 / (1 + expf(-cls_preds[tid]));
  
  // 判断当前锚点是否需要保留且置信度超过阈值
  if (anchor_mask[tid] == 1 && score > score_threshold) {
    // 原子操作递增过滤计数
    int counter = atomicAdd(filter_count, 1);
    
    // 计算锚点的z坐标
    float za = dev_anchors_pz[tid] + dev_anchors_dz[tid] / 2;
    
    // 解码网络输出得到预测的完整边界框信息
    float diagonal = sqrtf(dev_anchors_dx[tid] * dev_anchors_dx[tid] + dev_anchors_dy[tid] * dev_anchors_dy[tid]);
    float box_px = box_preds[tid * NUM_OUTPUT_BOX_FEATURE + 0] * diagonal + dev_anchors_px[tid];
    float box_py = box_preds[tid * NUM_OUTPUT_BOX_FEATURE + 1] * diagonal + dev_anchors_py[tid];
    float box_pz =box_preds[tid * NUM_OUTPUT_BOX_FEATURE + 2] * dev_anchors_dz[tid] + za;
    float box_dx =expf(box_preds[tid * NUM_OUTPUT_BOX_FEATURE + 3]) * dev_anchors_dx[tid];
    float box_dy =expf(box_preds[tid * NUM_OUTPUT_BOX_FEATURE + 4]) * dev_anchors_dy[tid];
    float box_dz =expf(box_preds[tid * NUM_OUTPUT_BOX_FEATURE + 5]) * dev_anchors_dz[tid];
    float box_ro =box_preds[tid * NUM_OUTPUT_BOX_FEATURE + 6] + dev_anchors_ro[tid];

	// 调整边界框的高度位置，把高度定位到底面上
    box_pz = box_pz - box_dz / 2;

    // 更新过滤后的边界框置信和方向数组
    filtered_box[counter * NUM_OUTPUT_BOX_FEATURE + 0] = box_px;
    filtered_box[counter * NUM_OUTPUT_BOX_FEATURE + 1] = box_py;
    filtered_box[counter * NUM_OUTPUT_BOX_FEATURE + 2] = box_pz;
    filtered_box[counter * NUM_OUTPUT_BOX_FEATURE + 3] = box_dx;
    filtered_box[counter * NUM_OUTPUT_BOX_FEATURE + 4] = box_dy;
    filtered_box[counter * NUM_OUTPUT_BOX_FEATURE + 5] = box_dz;
    filtered_box[counter * NUM_OUTPUT_BOX_FEATURE + 6] = box_ro;
    filtered_score[counter] = score;

    // 判断方向标签
    int direction_label;
    if (dir_preds[tid * 2 + 0] < dir_preds[tid * 2 + 1]) {
      direction_label = 1;
    } else {
      direction_label = 0;
    }
    filtered_dir[counter] = direction_label;


    // convrt normal box(normal boxes: x, y, z, w, l, h, r) to box(xmin, ymin,
    // xmax, ymax) for nms calculation First: dx, dy -> box(x0y0, x0y1, x1y0,
    // x1y1)
    //将三维框转换为二维框
    float corners[NUM_3D_BOX_CORNERS_MACRO] = {
        static_cast<float>(-0.5 * box_dx), static_cast<float>(-0.5 * box_dy),
        static_cast<float>(-0.5 * box_dx), static_cast<float>(0.5 * box_dy),
        static_cast<float>(0.5 * box_dx),  static_cast<float>(0.5 * box_dy),
        static_cast<float>(0.5 * box_dx),  static_cast<float>(-0.5 * box_dy)};

    // Second: Rotate, Offset and convert to point(xmin. ymin, xmax, ymax)
    float rotated_corners[NUM_3D_BOX_CORNERS_MACRO];
    float offset_corners[NUM_3D_BOX_CORNERS_MACRO];
    float sin_yaw = sinf(box_ro);
    float cos_yaw = cosf(box_ro);
    float xmin = FLOAT_MAX;
    float ymin = FLOAT_MAX;
    float xmax = FLOAT_MIN;
    float ymax = FLOAT_MIN;
    for (size_t i = 0; i < NUM_BOX_CORNERS; i++) {
      rotated_corners[i * 2 + 0] =
          cos_yaw * corners[i * 2 + 0] - sin_yaw * corners[i * 2 + 1];
      rotated_corners[i * 2 + 1] =
          sin_yaw * corners[i * 2 + 0] + cos_yaw * corners[i * 2 + 1];

      offset_corners[i * 2 + 0] = rotated_corners[i * 2 + 0] + box_px;
      offset_corners[i * 2 + 1] = rotated_corners[i * 2 + 1] + box_py;

      xmin = fminf(xmin, offset_corners[i * 2 + 0]);
      ymin = fminf(ymin, offset_corners[i * 2 + 1]);
      xmax = fmaxf(xmin, offset_corners[i * 2 + 0]);
      ymax = fmaxf(ymax, offset_corners[i * 2 + 1]);
    }
    // box_for_nms(num_box, 4)存储用于NMS操作的边界框
    box_for_nms[counter * NUM_BOX_CORNERS + 0] = xmin;
    box_for_nms[counter * NUM_BOX_CORNERS + 1] = ymin;
    box_for_nms[counter * NUM_BOX_CORNERS + 2] = xmax;
    box_for_nms[counter * NUM_BOX_CORNERS + 3] = ymax;
  }
}
```
1. 计算当前锚框的置信度score。
2. 判断置信度是否超过阈值及锚框是否需要保留,如果满足则进行过滤。
3. 原子加操作获取当前保留框的索引counter。
4. 解码RPN输出,获取完整的预测框信息。
5. 调整高度信息,把框调整到底面。
6. 更新过滤后的框/置信度/方向数组。
7. 将三维框转换为二维框,用于后续NMS计算。
8. 更新作为NMS输入的二维框数组box_for_nms。