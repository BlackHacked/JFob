# Ubuntu20.04 安装fcitx5输入法

> ubuntu 20.04.3下fcitx5 需要从flatpak安装，（由于qt版本，fcitx5-config只能安装在20.10上），中间出了各种问题，最后发现以下解决方案最好：

[](#序)序
-------


ubuntu 20.04.3下fcitx5 需要从flatpak安装，（由于qt版本，fcitx5-config只能安装在20.10上），中间出了各种问题，最后发现以下解决方案最好：

[](#安装flatpak)[安装flatpak](https://flatpak.org/setup/Ubuntu)
-----------------------------------------------------------

(建议使用官方ppa,版本较新)

<table><tbody><tr><td><pre><span>1</span><br><span>2</span><br><span>3</span><br><span>4</span><br><span>5</span><br></pre></td><td><pre><span>sudo add-apt-repository ppa:flatpak/stable</span><br><span>sudo apt update</span><br><span>sudo apt install flatpak</span><br><span>sudo apt install gnome-software-plugin-flatpak</span><br><span>flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo</span><br></pre></td></tr></tbody></table>

reboot

[](#安装fcitx5)安装fcitx5
---------------------

### [](#添加-flatub-仓库)添加 flatub 仓库,

<table><tbody><tr><td><pre><span>1</span><br><span>2</span><br><span>3</span><br><span>4</span><br></pre></td><td><pre><span># 添加 flatub 仓库, fcitx5-unstable 也会依赖一些这个仓库中的运行时软件包。</span><br><span>flatpak remote-add --user --if-not-exists flathub https://dl.flathub.org/repo/flathub.flatpakrepo</span><br><span># 可选部分: 如果你想要使用不稳定版本的fcitx5，也可以添加 fcitx5 非稳定仓库。</span><br><span># flatpak remote-add --user --if-not-exists fcitx5-unstable https://flatpak.fcitx-im.org/unstable-repo/fcitx5-unstable.flatpakrepo</span><br></pre></td></tr></tbody></table>
Note that the directories 

'/var/lib/flatpak/exports/share'
'/home/hanrui/.local/share/flatpak/exports/share'

are not in the search path set by the XDG_DATA_DIRS environment variable, so
applications installed by Flatpak may not appear on your desktop until the
session is restarted.
### 解决方案
export XDG_DATA_DIRS="/var/lib/flatpak/exports/share:/home/hanrui/.local/share/flatpak/exports/share:${XDG_DATA_DIRS:-/usr/local/share:/usr/share}"
写入bashrc

### [](#从flatpak安装fcitx)从flatpak安装fcitx

<table><tbody><tr><td><pre><span>1</span><br><span>2</span><br><span>3</span><br><span>4</span><br><span>5</span><br></pre></td><td><pre><span># 如果您使用的是旧版flatpak，在安装的时候会需要显示的指定软件仓库名字: flatpak install flathub org.fcitx.Fcitx5</span><br><span>flatpak install org.fcitx.Fcitx5</span><br><span># 安装 fcitx5 输入法引擎, 例如fcitx5-chinese-addons, or mozc</span><br><span>flatpak install org.fcitx.Fcitx5.Addon.ChineseAddons # 中文输入法</span><br><span># flatpak install org.fcitx.Fcitx5.Addon.Mozc			 # 日文输入法</span><br></pre></td></tr></tbody></table>
jklj
Looking for matches…
F: An error was encountered searching remote ‘flathub’ for ‘org.fcitx.Fcitx5.Addon.ChineseAddons’: Unable to load summary from remote flathub: While fetching https://dl.flathub.org/repo/summary.idx: [6] Couldn't resolve host name
F: An error was encountered searching remote ‘flathub’ for ‘org.fcitx.Fcitx5.Addon.ChineseAddons’: Unable to load summary from remote flathub: While fetching https://dl.flathub.org/repo/summary.idx: [6] Couldn't resolve host name
error: No remote refs found for ‘org.fcitx.Fcitx5.Addon.ChineseAddons’


### [](#安装ficitx5后端)安装ficitx5后端

<table><tbody><tr><td><pre><span>1</span><br></pre></td><td><pre><span>sudo apt install fcitx5-frontend-gtk2 fcitx5-frontend-gtk3 fcitx5-frontend-qt5</span><br></pre></td></tr></tbody></table>

之后从应用中打开fcitx5, 就可以愉快的使用了! (需要使用配置添加简体中文的拼音)

![](chrome-extension://cjedbglnccaioiolemnfhjncicchinao/fcitx5-ubuntu/image-20220211133357854.png "image-20220211133357854")

### [](#配置环境变量通用办法)[配置环境变量:(通用办法)](https://fcitx-im.org/wiki/Setup_Fcitx_5)




<table><tbody><tr><td><pre><span>1</span><br><span>2</span><br><span>3</span><br></pre></td><td><pre><span>export XMODIFIERS=@im=fcitx</span><br><span>export GTK_IM_MODULE=fcitx</span><br><span>export QT_IM_MODULE=fcitx</span><br></pre></td></tr></tbody></table>

### [](#reboot)reboot

### [](#在gnome-tweak中设置开机启动)在gnome-tweak中设置开机启动

[](#配置fcitx5)配置fcitx5
---------------------

### [](#安装自定义主题)安装自定义主题

[Adwaita-dark](https://github.com/escape0707/fcitx5-adwaita-dark)

<table><tbody><tr><td><pre><span>1</span><br></pre></td><td><pre><span>git clone https://github.com/escape0707/fcitx5-adwaita-dark.git ~/.local/share/fcitx5/themes/adwaita-dark</span><br></pre></td></tr></tbody></table>

之后在fcitx5设置界面中切换

### [](#安装词库)安装词库

[萌娘百科词库](https://github.com/outloudvi/mw2fcitx/releases/tag/20220114)

[维基百科词库](https://github.com/felixonmars/fcitx5-pinyin-zhwiki/releases/tag/0.2.3)

下载`*.dict`文件, 放置到 `~/.local/share/fcitx5/pinyin/dictionaries`

[](#一些bug的解决)一些bug的解决
---------------------

### [](#intellij-系列软件的-ide-中输入框位置不正确)IntelliJ 系列软件的 IDE 中输入框位置不正确

此问题的根本原因是 IDE 附带的 JBR 不正确，要处理此问题，需要：

1.  前往 [https://github.com/RikudouPatrickstar/JetBrainsRuntime-for-Linux-x64/releases](https://github.com/RikudouPatrickstar/JetBrainsRuntime-for-Linux-x64/releases) 下载 jbr 并解压到任意路径
2.  按照 [此指导](https://intellij-support.jetbrains.com/hc/en-us/articles/206544879-Selecting-the-JDK-version-the-IDE-will-run-under) 更改 IDE 的 JBR
    1.  ctrl+shift+A , 输入 [Choose Boot Java Runtime for the IDE](https://i.imgur.com/iAwt6b9.png)
    2.  选择[Add Custom Runtime option](https://i.imgur.com/f3k144X.png)
    3.  选择解压出的文件夹

[](#参考链接)参考链接：
--------------

[fcitx5官网](https://fcitx-im.org/wiki/Fcitx_5/zh-cn)

[安装fcitx5](https://fcitx-im.org/wiki/Install_Fcitx_5/zh-cn)

[如何现在就在 Ubuntu 20.04 下用上 Fcitx 5.0.2](https://plumz.me/archives/12285/)

[fcitx5 archwiki](https://wiki.archlinux.org/title/Fcitx5_(%E7%AE%80%E4%BD%93%E4%B8%AD%E6%96%87))


[Source](https://ouyen.github.io/fcitx5-ubuntu/)