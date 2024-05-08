---
tags: function
---
modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.cc
```cpp
void PointPillars::generateAnchors(float* anchors_px_, float* anchors_py_, float* anchors_pz_, float* anchors_dx_, float* anchors_dy_, float* anchors_dz_, float* anchors_ro_) {
  // zero clear
  for (int i = 0; i < NUM_ANCHOR_; i++) {
    anchors_px_[i] = 0;
    anchors_py_[i] = 0;
    anchors_pz_[i] = 0;
    anchors_dx_[i] = 0;
    anchors_dy_[i] = 0;
    anchors_dz_[i] = 0;
    anchors_ro_[i] = 0;
    box_anchors_min_x_[i] = 0;
    box_anchors_min_y_[i] = 0;
    box_anchors_max_x_[i] = 0;
    box_anchors_max_y_[i] = 0;
  }

  // 计算步长和偏移量
  float x_stride = PILLAR_X_SIZE_ * 2.0f;
  float y_stride = PILLAR_Y_SIZE_ * 2.0f;
  float x_offset = MIN_X_RANGE_ + PILLAR_X_SIZE_;
  float y_offset = MIN_Y_RANGE_ + PILLAR_Y_SIZE_;

  // 计算锚点的计数数组
  float anchor_x_count[NUM_ANCHOR_X_INDS_];//数组长度是一半x方向上的格数
  anchor_x_count[0] = 0;
  for (int i = 0; i < NUM_ANCHOR_X_INDS_; i++) {
    anchor_x_count[i] = static_cast<float>(i) * x_stride + x_offset;
  }
  float anchor_y_count[NUM_ANCHOR_Y_INDS_];
  anchor_y_count[0] = 0;
  for (int i = 0; i < NUM_ANCHOR_Y_INDS_; i++) {
    anchor_y_count[i] = static_cast<float>(i) * y_stride + y_offset;
  }

  float anchor_r_count[NUM_ANCHOR_R_INDS_];
  anchor_r_count[0] = 0;
  anchor_r_count[1] = M_PI / 2;

  // 生成锚点数组
  for (int y = 0; y < NUM_ANCHOR_Y_INDS_; y++) {
    for (int x = 0; x < NUM_ANCHOR_X_INDS_; x++) {
      for (int r = 0; r < NUM_ANCHOR_R_INDS_; r++) {
        int ind = y * NUM_ANCHOR_X_INDS_ * NUM_ANCHOR_R_INDS_ + x * NUM_ANCHOR_R_INDS_ + r;
        // 计算锚点的属性值并存储到对应的数组中
        anchors_px_[ind] = anchor_x_count[x];
        anchors_py_[ind] = anchor_y_count[y];
        anchors_ro_[ind] = anchor_r_count[r];
        anchors_pz_[ind] = -1 * SENSOR_HEIGHT_;
        anchors_dx_[ind] = ANCHOR_DX_SIZE_;
        anchors_dy_[ind] = ANCHOR_DY_SIZE_;
        anchors_dz_[ind] = ANCHOR_DZ_SIZE_;
      }
    }
  }
}

```