# NebulaGraph Importer - NebulaGraph Database 手册

> Documentation for NebulaGraph Database

[](https://github.com/vesoft-inc/nebula-docs-cn/edit/v3.6.0/docs-2.0/nebula-importer/use-importer.md "编辑此页")

NebulaGraph Importer（简称 Importer）是一款NebulaGraph的 CSV 文件单机导入工具，可以读取并导入多种数据源的 CSV 文件数据。

功能[¶](#_1 "Permanent link")
---------------------------

*   支持多种数据源，包括本地、S3、OSS、HDFS、FTP、SFTP。
*   支持导入 CSV 格式文件的数据。单个文件内可以包含多种 Tag、多种 Edge type 或者二者混合的数据。
*   支持同时连接多个 Graph 服务进行导入并且动态负载均衡。
*   支持失败后重连、重试。
*   支持多维度显示统计信息，包括导入时间、导入百分比等。统计信息支持打印在 Console 或日志中。

优势[¶](#_2 "Permanent link")
---------------------------

*   轻量快捷：不需要复杂环境即可使用，快速导入数据。

*   灵活筛选：通过配置文件可以实现对 CSV 文件数据的灵活筛选。

版本兼容性[¶](#_3 "Permanent link")
------------------------------

NebulaGraph Importer 版本和NebulaGraph内核的版本对应关系如下。

Note

Importer 4.0.0 对 Importer 进行了重做，性能得到了提高，但配置文件不兼容旧版本。建议使用新版 Importer。

更新说明[¶](#_4 "Permanent link")
-----------------------------

[Release notes](https://github.com/vesoft-inc/nebula-importer/releases/tag/v4.0.0)

前提条件[¶](#_5 "Permanent link")
-----------------------------

在使用 NebulaGraph Importer 之前，请确保：

*   已部署NebulaGraph服务。部署方式如下：
    
    *   [RPM/DEB 包安装](chrome-extension://cjedbglnccaioiolemnfhjncicchinao/4.deployment-and-installation/2.compile-and-install-nebula-graph/2.install-nebula-graph-by-rpm-or-deb/)
    
    *   [Docker Compose 部署](chrome-extension://cjedbglnccaioiolemnfhjncicchinao/2.quick-start/1.quick-start-overview/)
    
    *   [源码编译安装](chrome-extension://cjedbglnccaioiolemnfhjncicchinao/4.deployment-and-installation/2.compile-and-install-nebula-graph/1.install-nebula-graph-by-compiling-the-source-code/)

*   NebulaGraph中已创建 Schema，包括图空间、Tag 和 Edge type，或者通过参数`manager.hooks.before.statements`设置。

操作步骤[¶](#_6 "Permanent link")
-----------------------------
### 创建 CSV 文件[¶](#csv "Permanent link")
准备好待导入的 CSV 文件并配置 YAML 文件，即可使用本工具向NebulaGraph批量导入数据。
### 下载二进制包运行[¶](#_7 "Permanent link")
1.  在 [Release](https://github.com/vesoft-inc/nebula-importer/releases/tag/v4.0.0) 页面下载和安装二进制包，并添加执行权限。
    
    Note
    
    使用 RPM/DEB 包安装的文件路径为`/usr/bin/nebula-importer`。
    
2.  在`nebula-importer`的安装目录下，执行以下命令导入数据。
    
    `$ ./<binary_file_name> --config <yaml_config_file_path>` 
    

### 源码编译运行[¶](#_8 "Permanent link")

编译源码需要部署 Golang 环境。详情请参见 [Golang 环境搭建](https://github.com/vesoft-inc/nebula-importer/blob/release-4.0/docs/golang-install.md)。

1.  克隆仓库。
    
    `$ git clone -b release-4.0 https://github.com/vesoft-inc/nebula-importer.git` 
    
    Note
    
    请使用正确的分支。不同分支的 rpc 协议不同。
    
2.  进入目录`nebula-importer`。
    
3.  编译源码。
    
4.  开始导入数据。
    
    `$ ./bin/nebula-importer --config <yaml_config_file_path>` 
    

### Docker 方式运行[¶](#docker "Permanent link")

使用 Docker 可以不必在本地安装 Go 语言环境，只需要拉取 NebulaGraph Importer 的[镜像](https://hub.docker.com/r/vesoft/nebula-importer)，并将本地配置文件和 CSV 数据文件挂载到容器中。命令如下：

`$ docker pull vesoft/nebula-importer:<version>
$ docker run --rm -ti \
 --network=host \
 -v <config_file>:<config_file> \
 -v <data_dir>:<data_dir> \
 vesoft/nebula-importer:<version> \
 --config <config_file>` 

*   `<config_file>`：填写 YAML 配置文件的绝对路径。
*   `<data_dir>`：填写 CSV 数据文件的绝对路径。如果文件不在本地，请忽略该参数。
*   `<version>`：填写 Importer 的版本号，请填写`v4`。

Note

建议使用相对路径。如果使用本地绝对路径，请检查路径映射到 Docker 中的路径。

例如：

`$ docker pull vesoft/nebula-importer:v4
$ docker run --rm -ti \
 --network=host \
 -v /home/user/config.yaml:/home/user/config.yaml \
 -v /home/user/data:/home/user/data \
 vesoft/nebula-importer:v4 \
 --config /home/user/config.yaml` 

配置文件说明[¶](#_9 "Permanent link")
-------------------------------

NebulaGraph Importer 的 [Github](https://github.com/vesoft-inc/nebula-importer/tree/release-4.0/examples) 内提供多种示例配置文件。配置文件用来描述待导入文件信息、NebulaGraph服务器信息等。下文将分类介绍配置文件内的字段。

Note

如果用户下载的是二进制包，请手动创建配置文件。

### Client 配置[¶](#client "Permanent link")

Client 配置存储客户端连接NebulaGraph相关的配置。

示例配置如下：

`client:
 version: v3
 address: "192.168.1.100:9669,192.168.1.101:9669"
 user: root
 password: nebula
 concurrencyPerAddress: 10
 reconnectInitialInterval: 1s
 retry: 3
 retryInitialInterval: 1s` 

### Manager 配置[¶](#manager "Permanent link")

Manager 配置是连接数据库后的人为控制配置。

示例配置如下：

`manager:
 spaceName: basic_string_examples
 batch: 128
 readerConcurrency: 50
 importerConcurrency: 512
 statsInterval: 10s
 hooks:
 before:
 - statements:
 - |
 DROP SPACE IF EXISTS basic_string_examples;
 CREATE SPACE IF NOT EXISTS basic_string_examples(partition_num=5, replica_factor=1, vid_type=int);
 USE basic_string_examples;
 wait: 10s
 after:
 - statements:
 - |
 SHOW SPACES;` 

### Log 配置[¶](#log "Permanent link")

Log 配置是设置日志相关配置。

示例配置如下：

`log:
 level: INFO
 console: true
 files:
 - logs/nebula-importer.log` 

### Source 配置[¶](#source "Permanent link")

Source 配置中需要配置数据源信息、数据处理方式和 Schema 映射。

示例配置如下：

```
sources:
 - path: ./person.csv # 指定存储数据文件的路径。如果使用相对路径，则路径和当前配置文件目录拼接。也支持通配符文件名，例如：./follower-*.csv，请确保所有匹配的文件具有相同的架构。
#  - s3: # AWS S3
#      endpoint: endpoint    # 可选。S3 服务端点，如果使用 AWS S3 可以省略。
#      region: us-east-1     # 必填。S3 服务的区域。
#      bucket: gdelt-open-data    # 必填。S3 服务中的 bucket。
#      key: events/20190918.export.csv     # 必填。S3 服务中文件的 key。
#      accessKeyID: ""    # 可选。S3 服务的访问密钥。如果是公共数据，则无需配置。
#      accessKeySecret: ""     # 可选。S3 服务的密钥。如果是公共数据，则无需配置。
#  - oss:
#      endpoint: https://oss-cn-hangzhou.aliyuncs.com    # 必填。OSS 服务端点。
#      bucket: bucketName    # 必填。OSS 服务中的 bucket。
#      key: objectKey    # 必填。OSS 服务中文件的 object key。
#      accessKeyID: accessKey    # 必填。OSS 服务的访问密钥。
#      accessKeySecret: secretKey    # 必填。OSS 服务的秘钥。
#  - ftp:
#      host: 192.168.0.10    # 必填。FTP 服务的主机。
#      port: 21    # 必填。FTP 服务的端口。
#      user: user    # 必填。FTP 服务的用户名。
#      password: password    # 必填。FTP 服务的密码。
#      path: "/events/20190918.export.csv"    # FTP 服务中文件的路径。
#  - sftp:
#      host: 192.168.0.10    # 必填。SFTP 服务的主机。
#      port: 22    # 必填。SFTP 服务的端口。
#      user: user    # 必填。SFTP 服务的用户名。
#      password: password    # 可选。SFTP 服务的密码。
#      keyFile: keyFile    # 可选。SFTP 服务的 SSH 密钥文件路径。
#      keyData: keyData    $ 可选。SFTP 服务的 SSH 密钥文件内容。
#      passphrase: passphrase    # 可选。SFTP 服务的 SSH 密钥密码。
#      path: "/events/20190918.export.csv"    # 必填。SFTP 服务中文件的路径。
#  - hdfs:
#      address: "127.0.0.1:8020"    # 必填。HDFS 服务的地址。
#      user: "hdfs"    # 可选。HDFS 服务的用户名。
#      path: "/events/20190918.export.csv"    # 必填。HDFS 服务中文件的路径。
 batch: 256
 csv:
 delimiter: "|"
 withHeader: false
 lazyQuotes: false
 tags:
 - name: Person
 id:
 type: "STRING"
 function: "hash"
#       index: 0 
 concatItems:
 - person_
 - 0
 - _id
 props:
 - name: "firstName"
 type: "STRING"
 index: 1
 - name: "lastName"
 type: "STRING"
 index: 2
 - name: "gender"
 type: "STRING"
 index: 3
 nullable: true
 defaultValue: female
 - name: "birthday"
 type: "DATE"
 index: 4
 nullable: true
 nullValue: _NULL_
 - name: "creationDate"
 type: "DATETIME"
 index: 5
 - name: "locationIP"
 type: "STRING"
 index: 6
 - name: "browserUsed"
 type: "STRING"
 index: 7
 - path: ./knows.csv
 batch: 256
 edges:
 - name: KNOWS # person_knows_person
 src:
 id:
 type: "STRING"
 concatItems:
 - person_
 - 0
 - _id
 dst:
 id:
 type: "STRING"
 concatItems:
 - person_
 - 1
 - _id
 props:
 - name: "creationDate"
 type: "DATETIME"
 index: 2
 nullable: true
 nullValue: _NULL_
 defaultValue: 0000-00-00T00:00:00` 



配置主要包括以下几个部分：

*   指定数据源信息。
*   指定执行语句的批处理量。
*   指定 CSV 文件格式信息。
*   指定 Tag 的模式映射。
*   指定 Edge type 的模式映射。

Note

CSV 文件中列的序号从 0 开始，即第一列的序号为 0，第二列的序号为 1。

* * *

最后更新: October 17, 2023


[Source](https://docs.nebula-graph.com.cn/3.6.0/nebula-importer/use-importer/)