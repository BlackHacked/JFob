---
tags:functions
---
modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.cc
```cpp
void PointPillars::convertAnchors2BoxAnchors(float* anchors_px, float* anchors_py, float* anchors_dx, float* anchors_dy, float* box_anchors_min_x_, float* box_anchors_min_y_, float* box_anchors_max_x_,float* box_anchors_max_y_) {
  // flipping box's dimension
  //定义数组，长度是一半总格数
  float flipped_anchors_dx[NUM_ANCHOR_];
  flipped_anchors_dx[0] = 0;
  //定义数组，宽度是一半总格数
  float flipped_anchors_dy[NUM_ANCHOR_];
  flipped_anchors_dy[0] = 0;
  //遍历锚框
  //NUM_ANCHOR_X_INDS_是x轴格数的一半 432/2
  //NUM_ANCHOR_Y_INDS_是y轴格数的一半 496/2
  for (int x = 0; x < NUM_ANCHOR_X_INDS_; x++) {
    for (int y = 0; y < NUM_ANCHOR_Y_INDS_; y++) {
      int base_ind = x * NUM_ANCHOR_Y_INDS_ * NUM_ANCHOR_R_INDS_ + y * NUM_ANCHOR_R_INDS_;
      flipped_anchors_dx[base_ind + 0] = ANCHOR_DX_SIZE_;
      flipped_anchors_dy[base_ind + 0] = ANCHOR_DY_SIZE_;
      flipped_anchors_dx[base_ind + 1] = ANCHOR_DY_SIZE_;
      flipped_anchors_dy[base_ind + 1] = ANCHOR_DX_SIZE_;
    }
  }
  for (int x = 0; x < NUM_ANCHOR_X_INDS_; x++) {
    for (int y = 0; y < NUM_ANCHOR_Y_INDS_; y++) {
      for (int r = 0; r < NUM_ANCHOR_R_INDS_; r++) {
        int ind = x * NUM_ANCHOR_Y_INDS_ * NUM_ANCHOR_R_INDS_ + y * NUM_ANCHOR_R_INDS_ + r;
        box_anchors_min_x_[ind] = anchors_px[ind] - flipped_anchors_dx[ind] / 2.0f;
        box_anchors_min_y_[ind] = anchors_py[ind] - flipped_anchors_dy[ind] / 2.0f;
        box_anchors_max_x_[ind] = anchors_px[ind] + flipped_anchors_dx[ind] / 2.0f;
        box_anchors_max_y_[ind] = anchors_py[ind] + flipped_anchors_dy[ind] / 2.0f;
      }
    }
  }
}
```