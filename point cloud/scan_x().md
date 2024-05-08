---
tags: function
---
modules/perception/lidar/lib/detection/lidar_point_pillars/anchor_mask_cuda.cu
输出：dev_cumsum_along_x：Array for storing cumsum-ed 与算法无关，与cuda计算相关。后续算法实现中没有使用
```cpp
//参数：输出数组的指针、输入数组指针、数组的大小
__global__ void scan_x(int* g_odata, int* g_idata, int n) {
  extern __shared__ int temp[];  // allocated on invocation定义共享内存数组temp[],在调用时动态分配内存
  int thid = threadIdx.x;
  int bid = blockIdx.x;
  int bdim = blockDim.x;//block内的线程数
  int offset = 1;
//把输入数据加入到共享内存数组，每个线程加载两个元素
  temp[2 * thid] =
      g_idata[bid * bdim * 2 + 2 * thid];  // load input into shared memory
  temp[2 * thid + 1] = g_idata[bid * bdim * 2 + 2 * thid + 1];
//局部累加
  for (int d = n >> 1; d > 0; d >>= 1) {  // build sum in place up the tree
    __syncthreads();
    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      temp[bi] += temp[ai];
    }
    offset *= 2;
  }
 // clear the last element清除最后一个元素
  if (thid == 0) {
    temp[n - 1] = 0;
  }   
//全局累加
  for (int d = 1; d < n; d *= 2) {  // traverse down tree & build scan
    offset >>= 1;
    __syncthreads();
    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      int t = temp[ai];
      temp[ai] = temp[bi];
      temp[bi] += t;
    }
  }
//把累加结果写入输出数组
  __syncthreads();
  g_odata[bid * bdim * 2 + 2 * thid] =
      temp[2 * thid + 1];  // write results to device memory
  int second_ind = 2 * thid + 2;
  if (second_ind == bdim * 2) {
    g_odata[bid * bdim * 2 + 2 * thid + 1] =
        temp[2 * thid + 1] + g_idata[bid * bdim * 2 + 2 * thid + 1];
  } else {
    g_odata[bid * bdim * 2 + 2 * thid + 1] = temp[2 * thid + 2];
  }
}
```
int* g_odata: dev_cumsum_along_x
int* g_idata, dev_sparse_pillar_map
int n NUM_INDS_FOR_SCAN_