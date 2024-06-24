# Installation
```bash
conda install -c conda-forge jupyterlab
```

```bash
pip install jupyterlab
```

# config generation and remote access
```bash
>> jupyter-lab --generate-config (远程访问)
>> vim /xx/.jupyter/jupyter_lab_config.py
    c.ServerApp.ip = '0.0.0.0'  # 允许访问ip
    c.ServerApp.port = 8888   # 服务开启端口
    c.ServerApp.passwd = ''    # 登录密码   通过 > python -c " from notebook.auth import passwd; print(passwd()) " 生产 hash密码
    c.ServerApp.notebook_dir = '/data/d2/anaconda3/data' # jupyterlab 工作目录
    c.ServerApp.open_browser = False   # 服务启动时，是否打开浏览器
```