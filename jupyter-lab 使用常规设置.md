### 1、 jupyter-lab 添加不同内核
![[Pasted image 20240409085242.png]]
![1](https://img-blog.csdnimg.cn/bbec05b3e5bb45dea53897eb517a6656.png)

### jupyter-lab 远程访问服务

    >> jupyter-lab --generate-config (远程访问)
    >> vim /xx/.jupyter/jupyter_lab_config.py
        c.ServerApp.ip = '0.0.0.0'  
        c.ServerApp.port = 8888   
        c.ServerApp.passwd = ''    
        c.ServerApp.notebook_dir = '/data/d2/anaconda3/data' 
        c.ServerApp.open_browser = False

![[Pasted image 20240409085604.png]]

###  jupyter-lab修改工作目录
```
修改配置文件参数 root_dir 项目保存自定义路径即可切换工作目录
```
![1](https://img-blog.csdnimg.cn/8da782610677412ba208c97f68357e45.png)

### jupyter lab 卸载
```python
python3 -m pip uninstall -y jupyter jupyter_core jupyter-client jupyter-console jupyterlab_pygments qtconsole notebook nbconvert nbformat nbclassic nbclient jupyterlab-widgets jupyter-events jupyter-server jupyter-server-terminals
```