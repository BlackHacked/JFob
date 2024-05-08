# 基本配置情况
driver 版本：535.154.05
![[Pasted image 20240430105501.png]]
cuda 版本：V11.8.89
![[Pasted image 20240430105513.png]]

# pytorch
## 安装过程
```bash
conda create --name openmmlab
conda activate openmmlab
conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia
```
>安装命令来源
>https://pytorch.org/get-started/locally/
>![[Pasted image 20240430133836.png]]


>安装结果检查![[Pasted image 20240430143043.png]]

# 环境部署验证
![[Pasted image 20240430143119.png]]

# 使用 [MIM](https://github.com/open-mmlab/mim) 安装 [MMEngine](https://github.com/open-mmlab/mmengine)，[MMCV](https://github.com/open-mmlab/mmcv) 和 [MMDetection](https://github.com/open-mmlab/mmdetection)
1. 安装openmim
```bash
pip install -U openmim
```
![[Pasted image 20240430105832.png]]
如果出现以上报错，按提示依次安装
解决所有冲突后

2. 安装mmengine
```bash
mim install mmengine
```
未出现冲突或报错
3. 安装mmcv
```bash
mim install 'mmcv>=2.0.0rc4'
```
![[Pasted image 20240430144124.png]]
这一步可能会慢一点，耐心等待。
4. 安装mmdet
```bash
mim install 'mmdet>=3.0.0'
```
5. 安装 MMDetection3D
因为要开发并直接运行 mmdet3d，从源码安装它
如下命令是直接通过git拉取源码
```bash
git clone https://github.com/open-mmlab/mmdetection3d.git -b dev-1.x
```
"-b dev-1.x" 表示切换到 `dev-1.x` 分支。
```bash
cd mmdetection3d
pip install -v -e .
```
"-v" 指详细说明，或更多的输出
"-e" 表示在可编辑模式下安装项目，因此对代码所做的任何本地修改都会生效，从而无需重新安装。

# MMdetection3D安装验证
1. 下载配置文件和模型权重文件
```bash
mim download mmdet3d --config pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car --dest .
```
2. 验证
在没有图形化界面的情况下
```bash
python demo/pcd_demo.py demo/data/kitti/000008.bin pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth
```
输出
![[截屏2024-05-06 15.40.07.png]]
结果保存在./outputs/pred/000008.json

# 数据集准备
把kitti数据集放在

#


在 `pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py` 配置文件里，将 `objectSample` 的 `use_ground_plane` 参数设置为 `False` 的步骤如下：

1. 打开 `pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py` 配置文件。
    
2. 找到 `train_pipeline` 部分，其中应该包含一个数据增强或预处理步骤，比如 `ObjectSample`。
    
3. 在这个数据增强步骤里，将 `use_ground_plane` 参数设置为 `False`。例如：
    
    python
    
    Copy code
    
    `train_pipeline = [     # 其他预处理或数据增强步骤     dict(         type='ObjectSample',         use_ground_plane=False,         # 其他相关配置参数     ),     # 其他预处理或数据增强步骤 ]`

# 
```bash
python tools/analysis_tools/analyze_logs.py plot_curve work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/20240508_100452/vis_data/20240508_100452.json --keys loss_cls loss_bbox
```
