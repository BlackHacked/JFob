[[Three-Dimensional Scene Flow.pdf]]
1999 CVPR
2-dim optical flow
3-dim scene flow


## 场景流的解释-cited
>场景流表述了3D动态场景的motion field。想象一下，我们用诸如LiDAR的传感器在时间t-1采集某个自动驾驶场景的一个点云，作为S1。在时间 t，当场景发生了一些动态变化时，我们采集场景的另一个点云为 S2。在这里，点云 S2 的点数与点云 S1 的点数是不同的。而且这两个点云不是一一对应的。然后，对于 S1 中的点p，我们想要找到一个translational vector f，这个f可以最好地将点p移动到它在 S2 中的对应点。这些所有translational vector的集合就是场景流 calligraphic F。场景流是许多视觉任务的基础，例如point cloud registration、motion segmentation等等。