---
tags:functions
---
modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars.cc
```cpp
void PointPillars::putAnchorsInDeviceMemory() {

GPU_CHECK(cudaMemcpy(dev_box_anchors_min_x_, box_anchors_min_x_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_box_anchors_min_y_, box_anchors_min_y_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_box_anchors_max_x_, box_anchors_max_x_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_box_anchors_max_y_, box_anchors_max_y_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

  

GPU_CHECK(cudaMemcpy(dev_anchors_px_, anchors_px_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_py_, anchors_py_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_pz_, anchors_pz_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_dx_, anchors_dx_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_dy_, anchors_dy_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_dz_, anchors_dz_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

GPU_CHECK(cudaMemcpy(dev_anchors_ro_, anchors_ro_, NUM_ANCHOR_ * sizeof(float), cudaMemcpyHostToDevice));

}
```