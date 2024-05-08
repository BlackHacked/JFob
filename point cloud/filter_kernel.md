```cpp
__global__ void filter_kernel(
    const float* box_preds, 
    const float* cls_preds, 
    const float* dir_preds,
    const int* anchor_mask, 
    const float* dev_anchors_px,
    const float* dev_anchors_py, 
    const float* dev_anchors_pz,
    const float* dev_anchors_dx, 
    const float* dev_anchors_dy,
    const float* dev_anchors_dz, 
    const float* dev_anchors_ro,
    float* filtered_box, 
    float* filtered_score, 
    int* filtered_label,
    int* filtered_dir, 
    float* box_for_nms, 
    int* filter_count,
    const float float_min, 
    const float float_max, 
    const float score_threshold,
    const int num_box_corners, 
    const int num_output_box_feature,
    const int num_class) {
  // boxes ([N, 7] Tensor): normal boxes: x, y, z, w, l, h, r
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  // sigmoid function
  // 将每个anchor选出其最大类别置信度分数，并存储
  //存储其top_score置信度评分，以及top_lable：多中选一的具体类别
  float top_score = 0;
  int top_label = 0;
  for (int i = 0; i < num_class; ++i) {
    float score = 1 / (1 + expf(-cls_preds[tid * num_class + i]));
    if (score > top_score) {
      top_score = score;
      top_label = i;
    }
  }

//筛选条件
//anchor_mask是1的anchor：该锚框中有点
//top_score大于score_threshold：该锚框置信度最高类别的置信大于阈值
  if (anchor_mask[tid] == 1 && top_score > score_threshold) {
    int counter = atomicAdd(filter_count, 1);
    float za = dev_anchors_pz[tid] + dev_anchors_dz[tid] / 2;

    // decode network output
    //解码锚框的7个bbox值
    float diagonal = sqrtf(dev_anchors_dx[tid] * dev_anchors_dx[tid] +
                           dev_anchors_dy[tid] * dev_anchors_dy[tid]);
    float box_px = box_preds[tid * num_output_box_feature + 0] * diagonal +
                   dev_anchors_px[tid];
    float box_py = box_preds[tid * num_output_box_feature + 1] * diagonal +
                   dev_anchors_py[tid];
    float box_pz =
        box_preds[tid * num_output_box_feature + 2] * dev_anchors_dz[tid] + za;
    float box_dx =
        expf(box_preds[tid * num_output_box_feature + 3]) * dev_anchors_dx[tid];
    float box_dy =
        expf(box_preds[tid * num_output_box_feature + 4]) * dev_anchors_dy[tid];
    float box_dz =
        expf(box_preds[tid * num_output_box_feature + 5]) * dev_anchors_dz[tid];
    float box_ro =
        box_preds[tid * num_output_box_feature + 6] + dev_anchors_ro[tid];
//
    box_pz = box_pz - box_dz / 2;

    // 将筛选后的边界框及类别标签等存储进filter_box
    filtered_box[counter * num_output_box_feature + 0] = box_px;
    filtered_box[counter * num_output_box_feature + 1] = box_py;
    filtered_box[counter * num_output_box_feature + 2] = box_pz;
    filtered_box[counter * num_output_box_feature + 3] = box_dx;
    filtered_box[counter * num_output_box_feature + 4] = box_dy;
    filtered_box[counter * num_output_box_feature + 5] = box_dz;
    filtered_box[counter * num_output_box_feature + 6] = box_ro;
    filtered_score[counter] = top_score;
    filtered_label[counter] = top_label;

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
    float xmin = float_max;
    float ymin = float_max;
    float xmax = float_min;
    float ymax = float_min;
    for (size_t i = 0; i < num_box_corners; ++i) {
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
    // box_for_nms(num_box, 4)
    box_for_nms[counter * num_box_corners + 0] = xmin;
    box_for_nms[counter * num_box_corners + 1] = ymin;
    box_for_nms[counter * num_box_corners + 2] = xmax;
    box_for_nms[counter * num_box_corners + 3] = ymax;
  }
}





