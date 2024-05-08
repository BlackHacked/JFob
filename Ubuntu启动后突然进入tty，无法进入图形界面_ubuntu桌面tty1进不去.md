
**记录：**今晚删除了里面的python3.10，没有关闭[Ubuntu](https://so.csdn.net/so/search?q=Ubuntu&spm=1001.2101.3001.7020)就直接关闭了电脑，后面再看的时候发现Ubuntu开启后无法进入图形界面，直接是tty1界面。网上查找发现，可能是因为误删了图形界面的相关文件，不能正常连网，所以无法进入。先不慌，试试按以下方法处理。

**千万不要尝试删除Ubuntu自带的python版本！！**

**解决方法：**

**第一步：先连接网络**

    #尝试连接网络sudo dhclient eth0 #如果显示cannot find device "eth0"，可能是“eth0”已经改名字了#可以直接执行以下命令重新联网sudo dhclient#测试是否成功连接网络ping www.baidu.com

**第二步：若上一步能成功ping到，安装桌面**

    #更新sudo apt-get update#重新安装桌面sudo apt-get install ubuntu-desktop#重启reboot

成功解决\\(^o^)/。


[Source](https://blog.csdn.net/m0_62600040/article/details/135258796)