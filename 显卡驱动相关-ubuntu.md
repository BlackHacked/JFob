# 卸载
```bash
sudo apt-get remove --purge '^nvidia-.*'
dpkg -l | grep -i nvidia
```
![[Pasted image 20240409093428.png]]
检查是否所有包均已卸载干净，如有未卸载的包，手动卸载
```bash
sudo apt-get remove --purge xxx
```
重启
```bash
reboot
```

# 安装
## 离线安装
1. 搜索nvidia driver download https://www.nvidia.com/download/index.aspx
2. 找到Nvidia显卡产品序列
```bash
lspci | grep -i nvidia
```
3. 选择对应的显卡产品序列、操作系统和语言，下载离线安装包![[Pasted image 20240409094244.png]]
4. 进入安装包对应目录
```bash
chmod +x NVIDIA-Linux-x86_64-XXX.XX.XX.run  
sudo ./NVIDIA-Linux-x86_64-XXX.XX.XX.run
```
5. 跟随提示信息完成安装
 ![](https://safran-navigation-timing.b-cdn.net/wp-content/uploads/2022/09/1.jpeg)
![](https://safran-navigation-timing.b-cdn.net/wp-content/uploads/2022/09/2.jpeg) ![](https://safran-navigation-timing.b-cdn.net/wp-content/uploads/2022/09/3.jpeg) 

 ![](https://safran-navigation-timing.b-cdn.net/wp-content/uploads/2022/09/5.jpeg)
6. 重启
```bash
reboot
```
重启后打开终端，输入命令查看驱动是否安装成功。🥰

    
    nvidia-smi
    

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ee7c30206f9f4fffbce82ad521ac05ec.png#pic_center)
## 在线安装
### 方法（一）
1. 查看硬件适用的显卡驱动版本
```bash
sudo ubuntu-drivers list
```
返回显卡驱动列表
```
nvidia-driver-418-server
nvidia-driver-515-server
nvidia-driver-525-server
nvidia-driver-450-server
nvidia-driver-515
nvidia-driver-525
```
2. 安装
```
sudo ubuntu-drivers install
```
3. 重启
```bash
reboot
```
重启后打开终端，输入命令查看驱动是否安装成功。🥰

    
    nvidia-smi
    

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ee7c30206f9f4fffbce82ad521ac05ec.png#pic_center)

### 方法（二）
```
sudo ubuntu-drivers autoinstall
```
重启
```bash
reboot
```
重启后打开终端，输入命令查看驱动是否安装成功。🥰

    
    nvidia-smi
    

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ee7c30206f9f4fffbce82ad521ac05ec.png#pic_center)

### 方法（三）
#### 图形化界面安装

使用图形化界面安装驱动，首先打开ubuntu里面的`附加驱动`的应用程序。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/39c76e00b4ce452184ee89d3d59cb449.png#pic_center)

稍等一下，附加驱动就会加载出来当前机器能安装的驱动。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/1bccc15219744ff09a8454fb14ba2212.png#pic_center)

> 😨选择合适的驱动，如果是**消费级显卡**个人**不建议**安装带`open`的驱动；如果是**图形化**界面，**不要**安装带`server`的驱动。

选中想要安装的驱动后，点击应用更改，开始安装。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/41f6500815af410c9c744ddb7ea9a345.png#pic_center)

在安装脚本执行完成后，重启电脑。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/3b14e0a450794c60a8bbe55fd000194e.png#pic_center)

重启后打开终端，输入命令查看驱动是否安装成功。🥰

    
    nvidia-smi
    

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ee7c30206f9f4fffbce82ad521ac05ec.png#pic_center)


# 其他配置
## 禁止内核更新
```bash
vi /etc/apt/apt.conf.d/10periodic
vi /etc/apt/apt.conf.d/20auto-upgrades
```
后面部分全部改成 “0”
```bash
reboot
```