---
tags:functions
---
modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.cc
```cpp
void PointPillars::initAnchors() {
  // allocate memory for anchors
 //为变量创建数组，长度是NUM_ANCHOR_，即总格数的一半
  anchors_px_ = new float[NUM_ANCHOR_];//锚框中心x坐标
  anchors_py_ = new float[NUM_ANCHOR_];//锚框中心y坐标
  anchors_pz_ = new float[NUM_ANCHOR_];//锚框中心z坐标
  anchors_dx_ = new float[NUM_ANCHOR_];//锚框在x方向上的尺寸
  anchors_dy_ = new float[NUM_ANCHOR_];//锚框在y方向上的尺寸
  anchors_dz_ = new float[NUM_ANCHOR_];//锚框在z方向上的尺寸
  anchors_ro_ = new float[NUM_ANCHOR_];//旋转角度
  box_anchors_min_x_ = new float[NUM_ANCHOR_];//anchor对应2D框的最小x坐标
  box_anchors_min_y_ = new float[NUM_ANCHOR_];//anchor对应2D框的最小y坐标
  box_anchors_max_x_ = new float[NUM_ANCHOR_];//anchor对应2D框的最大x坐标
  box_anchors_max_y_ = new float[NUM_ANCHOR_];//anchor对应2D框的最大y坐标
  // deallocate these memories in deconstructor

  generateAnchors(anchors_px_, anchors_py_, anchors_pz_, anchors_dx_, anchors_dy_, anchors_dz_, anchors_ro_);

  convertAnchors2BoxAnchors(anchors_px_, anchors_py_, anchors_dx_, anchors_dy_, box_anchors_min_x_, box_anchors_min_y_, box_anchors_max_x_,
                            box_anchors_max_y_);

  putAnchorsInDeviceMemory();
}
```

```cpp
NUM_ANCHOR_X_INDS_(GRID_X_SIZE_ * 0.5),
NUM_ANCHOR_Y_INDS_(GRID_Y_SIZE_ * 0.5),
NUM_ANCHOR_R_INDS_(2),
NUM_ANCHOR_(NUM_ANCHOR_X_INDS_ * NUM_ANCHOR_Y_INDS_ * NUM_ANCHOR_R_INDS_),
```
