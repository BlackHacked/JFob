# 环境配置
## 基本配置信息
driver 版本：535.154.05
![[Pasted image 20240430105501.png]]
cuda 版本：V11.8.89
![[Pasted image 20240430105513.png]]

# 创建虚拟环境
```bash
conda create --name openmmlab python=3.8 -y
conda activate openmmlab
```
## 检查shell level
在**虚拟环境内**输入命令
```bash
conda info
```
![[未命名_副本.png]]
shell level 是 1 则可以进行下一步。
## pytorch安装
## 安装过程
```bash
conda create -n openmmlab python=3.8 -y
conda activate openmmlab
conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia
```
>安装命令来源
>https://pytorch.org/get-started/locally/
>![[Pasted image 20240430133836.png]]


>安装结果检查![[Pasted image 20240430143043.png]]

## 环境验证
```bash
python -c "import torch; print(torch.cuda.is_available(), torch.version.cuda, torch.__version__)"
```
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
4. 安装mmdet
```bash
mim install 'mmdet>=3.0.0'
```
5. 安装 MMDetection3D(验证)
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

## MMdetection3D安装验证
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
把kitti数据集放在mmdetection3d/data/kitti/

# 训练
## 超参数编辑
```bash
cd configs/pointpillars/
vim pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py
```

以下参数设置供参考
```python
lr = 0.001
epoch_num = 120
train_cfg = dict(by_epoch=True, max_epochs=epoch_num, val_interval=2)
```
## 命令
```shell
python ./tools/train.py ./configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py
```

>建议使用nohup/tmux


## 恢复训练
### 模板
```shell
python tools/train.py ${CONFIG_FILE} --resume work_dirs/lenet5_mnist/epoch_4.pth
```

### 命令
```bash
python ./tools/train.py ./configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py --resume work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/epoch_80.pth
```

        run: |
          git config --global user.email "your-email"
          git config --global user.name "your-id"
          git config --global url.https://${{ secrets.PAT }}@github.com/.insteadOf https://github.com/


## 分类和边界框损失曲线
```bash
python tools/analysis_tools/analyze_logs.py plot_curve work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/20240508_100452/vis_data/20240508_100452.json --keys loss_cls loss_bbox
```
> 需要把work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/20240508_100452/vis_data/20240508_100452.json 修改为训练时间对应的路径

## 问题解决
>1.  AssertionError: `use_ground_plane` is True but find plane is None

在 `pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py` 配置文件里，将 `objectSample` 的 `use_ground_plane` 参数设置为 `False` 的步骤如下：
1. 打开 `pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py` 配置文件。
2. 找到 `train_pipeline` 部分，其中应该包含一个数据增强或预处理步骤，比如 `ObjectSample`。
3. 将 `use_ground_plane` 参数设置为 `False`。
```python
    train_pipeline = [      
    dict(         type='ObjectSample',         use_ground_plane=False,         # 其他相关配置参数     
    ),    
      ]`
```

# 测试
## 命令

```bash
python tools/test.py ${CONFIG_FILE} ${CHECKPOINT_FILE} [--out ${RESULT_FILE}] [--eval ${EVAL_METRICS}] [--show] [--show-dir ${SHOW_DIR}]
```

```python
 python tools/test.py configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py /data/Projects/python_workplace/mmdetection3d/work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/epoch_1.pth --work-dir ./data/result_output/out_dir/3dssd.pkl  --cfg-options 'show=True' 'out_dir=./data/result_output/show_result'
```

```python
python tools/test.py configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/epoch_40.pth --work-dir ./data/result_output/out_dir/3dssd.pkl  --cfg-options 'show=True' 'out_dir=./data/result_output/show_result'
```
## 命令涉及到的文件路径
#### config file 配置文件
文件名：pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py
示例路径：mmdetection/configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py

#### checkpoint 检查点文件
文件名：epoch_1.pth
示例路径：mmdetection3d/work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/epoch_1.pth

#### result 测试结果
文件名：3dssd.pkl
示例路径：（路径通过 --work-dir指定）mmdetection3d/data/result_output/out_dir/3dssd.pkl

# 推理
## test结果可视化
```bash
python tools/test.py configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py /data/Projects/python_workplace/mmdetection3d/work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/epoch_1.pth --show --show-dir /data/result_output/show_result
```

```bash

```


## 单帧推理
```bash
python demo/pcd_demo.py demo/data/kitti/000008.bin configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py /data/Projects/python_workplace/mmdetection3d/work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/epoch_80.pth --show
```

```bash
python demo/pcd_demo.py demo/data/kitti/000008.bin configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py work_dirs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class/epoch_40.pth --show
```

