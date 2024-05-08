### 手动安装

> Github容易抽风，下载安装包可能会很慢，或者中途失败，因此我是用手动安装的方式，自动安装比较简单稍后介绍。

来到https://github.com/coder/code-server  
点击Releases来到发行物页面：  
![](https://img-blog.csdnimg.cn/img_convert/b6cf1c03454b0e7026ad0642216d186b.png)  
因为我们是Docker的Ubuntu选择下载这个：  
![](https://img-blog.csdnimg.cn/img_convert/3edfd334f358e053b54cdc8a2d2ba693.png)  
上传我们的安装包，在其目录下运行 ：

    dpkg -i code-server_4.19.0_amd64.deb.
    

安装完后运行：

    code-server
    

第一次运行，会在.config/code-server/目录下生成config.yaml 文件:

    bind-addr: 127.0.0.1:8080
    auth: password
    password: xxxxx
    cert: false
    

bind-addr是绑定的地址，因为是Docker容器都会与服务器本机有端口映射，比如20031:4091，前者代表服务器的端口，后者代表容器的端口。

这里将bind-addr修改成0.0.0.0:4091 ,这样当我们访问地址：`服务器IP：20031`的时候，会帮我们转到容器的4091端口，实现网站的访问。

password表示登录密码（按需修改），auth表示登录方式，不需要改。

重新上传config.yaml文件或者直接修改后，重新运行：

    code-server
    

这一次浏览器打开`服务器IP：20031`，见到美妙的画面：

![](https://img-blog.csdnimg.cn/img_convert/7ba695485647c2da929e755e34638296.png)  
接下来就是享用你的代码时间  
![](https://img-blog.csdnimg.cn/img_convert/3894378fffc5f9f0b05a294e6e439248.png)
