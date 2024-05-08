# 星形搜索 star shaped search
![[Pasted image 20231020131825.png]]

```cpp
// 星形搜索路缘检测
if (s > 1) { // 当点数大于等于2时进行检测
  
  float kdev = params::kdev_param; // 当前点斜率与平均斜率偏差的系数（控制偏差的敏感度）
  float kdist = params::kdist_param; // 距离权重系数：相邻点距离的系数
  int dmin = params::dmin_param; // 最小点数量要求:开始动态评估的最小点数
  
  float avg = 0, dev = 0, nan = 0; // 平均斜率、全局平均斜率、NaN数
  float ax, ay, bx, by, slp; // 相邻点a、b及斜率 
  
  bx = beams[tid].p[0].r; // 第一个点的r坐标（径向）
  by = array2D[beams[tid].p[0].id].p.z; // 第一个点的z坐标（高度）

  for (int i = 1; i < s; i++) { 
    // 更新相邻两点的坐标
    ax = bx; 
    bx = beams[tid].p[i].r;
    ay = by;
    by = array2D[beams[tid].p[i].id].p.z;
    
    slp = slope(ax, ay, bx, by); // 计算相邻两点的斜率
    
    if (isnan(slp)) // NaN值处理
      nan++;  
    else { //更新到当前点为止的平均斜率和平均斜率变化量
      // 更新当前点的平均斜率
      avg *= i - nan - 1;   //上一个点的平均值*到上个点为止的点数
      avg += slp;//加当前点斜率
      avg /= i - nan;//除到当前点为止的点数
      
      // 更新到当前点为止的平均斜率变化量（到当前点为止的每个点的斜率与到该点为止的平均斜率的差值的平均值）
      dev *= i - nan - 1;
      dev += abs(slp - avg);
      dev /= i - nan; 
    }
    
    // 使用固定阈值 + 动态阈值判断边缘
    if (slp > slope_param || 
        (i > dmin && (slp*slp - avg*avg) * kdev * ((bx - ax) * kdist) > dev)) {
        
      array2D[beams[tid].p[i].id].isCurbPoint = 2; // 标记为路缘点
      break; // 找到边缘,退出循环
  }
}
```

```cpp
//x-zero路缘检测
//遍历每一条线
for (int i = 0; i < index; i++)
{
//遍历整条线上的点
for (int j = 1; j < indexArray[i]; j++)
{
array3D[i][j].newY = array3D[i][j-1].newY + 0.0100;
}
//根据每个当前点选择对应的两个评估点p2和p3
//p2是距离当前点1/2curbPoints个点的点
//p3是距离当前点curbPoints个点的点
for (int j = params::curbPoints; j <= (indexArray[i] - 1) - params::curbPoints; j++)
{
p2 = j + params::curbPoints / 2;
p3 = j + params::curbPoints;

//计算p3和当前点的距离d
d = sqrt(
pow(array3D[i][p3].p.x - array3D[i][j].p.x , 2) +
pow(array3D[i][p3].p.y - array3D[i][j].p.y , 2));

//当d不超过5米时
if (d < 5.0000)
{
//计算三个点彼此之间的距离
x1 = sqrt(
pow(array3D[i][p2].newY - array3D[i][j].newY , 2) +
pow(array3D[i][p2].p.z - array3D[i][j].p.z , 2));
x2 = sqrt(
pow(array3D[i][p3].newY - array3D[i][p2].newY, 2) +
pow(array3D[i][p3].p.z - array3D[i][p2].p.z , 2));
x3 = sqrt(
pow(array3D[i][p3].newY - array3D[i][j].newY , 2) +
pow(array3D[i][p3].p.z - array3D[i][j].p.z , 2));

//计算p2角度参数
bracket = (pow(x3, 2) - pow(x1, 2) - pow(x2, 2)) / (-2 * x1 * x2);
if (bracket < -1)
bracket = -1;
else if (bracket > 1)
bracket = 1;
alpha = acos(bracket) * 180 / M_PI;

//判断条件
	//1. P2点的夹角参数小于阈值
	//2. 当前点与p2,或p2与p3的高度差大于阈值
	//3. 当前点与p3的高度差大于0.05
//判断条件同时满足，判断为路缘点
if (alpha <= params::angleFilter1 &&
(abs(array3D[i][j].p.z - array3D[i][p2].p.z ) >= params::curbHeight ||
abs(array3D[i][p3].p.z - array3D[i][p2].p.z ) >= params::curbHeight) &&
abs(array3D[i][j].p.z - array3D[i][p3].p.z ) >= 0.05)
{
array3D[i][p2].isCurbPoint = 2;
}
}
}
}
```

```cpp
//z-zero路缘检测
//遍历每一条线
for (int i = 0; i < index; i++){
//遍历整条线上的点
for (int j = params::curbPoints; j <= (indexArray[i] - 1) - params::curbPoints; j++)
{

//计算当前点前curbPoints个点与当前点后curbPoints个点，两个点之间的距离
d = sqrt(
pow(array3D[i][j+params::curbPoints].p.x - array3D[i][j-params::curbPoints].p.x, 2) +
pow(array3D[i][j+params::curbPoints].p.y - array3D[i][j-params::curbPoints].p.y, 2));

//如果距离小于5米
if (d < 5.0000)
{
//初始化max1和max2为当前点的高度(z)
//初始化4个变量是0
max1 = max2 = abs(array3D[i][j].p.z);
va1 = va2 = vb1 = vb2 = 0;

//遍历当前点到前curbPoints个点之间的所有点
for (int k = j - 1; k >= j - params::curbPoints; k--)
{
//va1：累加这些点与当前点x值的差值
//va2：累加这些点与当前点y值的差值
va1 = va1 + (array3D[i][k].p.x - array3D[i][j].p.x);
va2 = va2 + (array3D[i][k].p.y - array3D[i][j].p.y);
//max1是遍历到所有点的最大高度值z
if (abs(array3D[i][k].p.z) > max1)
max1 = abs(array3D[i][k].p.z);
}

//遍历当前点到后curbPoints个点之间的所有点
for (int k = j + 1; k <= j + params::curbPoints; k++)
{
//vb1：累加这些点与当前点x值的差值
//vb2：累加这些点与当前点y值的差值
vb1 = vb1 + (array3D[i][k].p.x - array3D[i][j].p.x );
vb2 = vb2 + (array3D[i][k].p.y - array3D[i][j].p.y );
//max2是遍历到所有点的最大高度值z
if (abs(array3D[i][k].p.z ) > max2)
max2 = abs(array3D[i][k].p.z);
}
//均值
//va1：当前点与前curbPoints个点之间x的差值的均值
//va2：当前点与前curbPoints个点之间y的差值的均值
//vb1：当前点与后curbPoints个点之间x的差值的均值
//vb2：当前点与后curbPoints个点之间y的差值的均值
va1 = (1 / (float)params::curbPoints) * va1;
va2 = (1 / (float)params::curbPoints) * va2;
vb1 = (1 / (float)params::curbPoints) * vb1;
vb2 = (1 / (float)params::curbPoints) * vb2;

//利用点积和反三角函数算夹角
bracket = (va1 * vb1 + va2 * vb2) / (sqrt(pow(va1, 2) + pow(va2, 2)) * sqrt(pow(vb1, 2) + pow(vb2, 2)));
if (bracket < -1)
bracket = -1;
else if (bracket > 1)
bracket = 1;
alpha = acos(bracket) * 180 / M_PI;

//判断条件
	//1. 夹角小于阈值
	//2. 当前点与前curbPoints个点中最高点的高度差或当前点与后curbPoints个点中最高点的高度差大于阈值
	//3. 前curbPoints个点中最高点和后curbPoints个点中最高点的高度差大于0.05
//判断条件同时满足，判断为路缘点
if (alpha <= params::angleFilter2 &&
(max1 - abs(array3D[i][j].p.z ) >= params::curbHeight ||
max2 - abs(array3D[i][j].p.z) >= params::curbHeight) &&
abs(max1 - max2) >= 0.05)
{
array3D[i][j].isCurbPoint = 2;
}
}
}
}
```


The current list of frames is:
Frame base_link exists with parent gps.
Frame gps exists with parent map.
Frame map_zala_0 exists with parent map.
Frame laser exists with parent base_link.
Frame duro_gps_imu exists with parent base_link.
Frame map_gyor_0 exists with parent map.
Frame velodyne_right exists with parent base_link.
Frame velodyne_left exists with parent base_link.
Frame left_os1/os1_lidar exists with parent left_os1/os1_sensor.
Frame left_os1/os1_sensor exists with parent base_link.
Frame right_os1/os1_imu exists with parent right_os1/os1_sensor.
Frame right_os1/os1_sensor exists with parent base_link.
Frame right_os1/os1_lidar exists with parent right_os1/os1_sensor.
Frame radar exists with parent base_link.
Frame zed_camera_front exists with parent base_link.
Frame rslidar exists with parent map.
Frame duro_gps exists with parent base_link.
Frame left_os1/os1_imu exists with parent left_os1/os1_sensor.