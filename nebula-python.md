工程文件位置：http://10.58.136.133:4091/?folder=/home/hanrui/nebula/fault-track
### Nebula Graph 与 Python 的数据操作和导入

#### 1. 函数定义与数据预处理
- **`process_file_id(data: pd.DataFrame) -> pd.DataFrame`**:
    - 清理标识列中的逗号。
    - 解析每行的缩进级别以确定层级。
    - 插入层级列和节点名称列。
    - 如果存在，清理状态和位置列中的空格。
#### 2. 标签（Tag）和顶点（Vertex）操作
- **`tag_insertion(session, data: pd.DataFrame)`**:
    - 对数据按缩进层级分组，并为每个层级创建一个标签（Tag）。
    - 检查每个创建操作是否成功，若失败则打印错误信息。
- **`vertex_import_by_type(session, data: pd.DataFrame)`**:
    - 同样按层级分组，为每个数据行插入顶点，使用层级信息作为顶点类型。
    - 检查顶点插入操作是否成功，若失败则打印错误信息。

#### 3. 边（Edge）操作
- **`create_edge(session, edge_type)`**:
    - 创建边类型（Edge Type）。
    - 捕获并打印任何错误。
- **`insert_tree_edges(session, data: pd.DataFrame)`**:
    - 根据BOM（Bill of Materials）树结构导入边。
    - 遍历数据集，为每个节点找到父节点并建立边。
    - 捕获并打印任何错误。

#### 4. 数据导入流程
- **`insert_bom(session, file_path, space_name, edge_type)`**:
    - 读取并处理CSV文件数据。
    - 指定工作空间，并执行导入标签、顶点和边的操作。

#### 5. Nebula Graph 连接配置和管理
- **配置和初始化连接池**:
    - 定义配置并初始化Nebula Graph的连接池。
    - 检查连接是否成功初始化。
- **使用会话上下文**:
    - 使用`session_context`管理数据库会话，确保会话自动释放。
    - 指定文件路径、空间名称和边类型来执行BOM的导入。
    - 关闭连接池。
```python
from nebula3.gclient.net import ConnectionPool
from nebula3.Config import Config
import pandas as pd
from typing import Dict
from nebula3.data.ResultSet import ResultSet
import json

def process_file_id(data: pd.DataFrame) -> pd.DataFrame:
    def clean_identification(ident):
        return ident.replace(',', ' ')
    data['标识'] = data['标识'].apply(clean_identification)

    def parse_indentation_levels(row):
        leading_spaces = len(row['标识']) - len(row['标识'].lstrip(' '))
        return leading_spaces // 4 + 1
    data['Indentation Level'] = data.apply(parse_indentation_levels, axis=1)

    data['Node Name'] = data.apply(lambda row: f"Node_{row['标识'].strip()}_{row['Indentation Level']}", axis=1)

    if '状态' in data.columns and '位置' in data.columns:
        data['状态'] = data['状态'].apply(lambda x: f"{x.replace(' ', '')}")
        data['位置'] = data['位置'].apply(lambda x: f"{x.replace(' ', '')}")
    return data

def tag_insertion(session,data: pd.DataFrame):
    grouped_data = data.groupby('Indentation Level')
    for level, group in grouped_data:
        tag_name = f"BoM_Level_{level}"
        response = session.execute(f"CREATE TAG IF NOT EXISTS {tag_name}() ;")
        if not response.is_succeeded():
            error_message = response.error_msg().encode('latin-1', errors='replace')
            print(f"Failed to insert tag {tag_name}: {error_message}")

def vertex_import_by_type(session, data: pd.DataFrame):
    grouped_data = data.groupby('Indentation Level')
    for level, group in grouped_data:
        tag_name = f"BoM_Level_{level}"
        for index, row in group.iterrows():
            properties = f"{row['Indentation Level']}"
            response = session.execute(f"INSERT VERTEX {tag_name} () VALUES '{row['标识']}': ();" )
            if not response.is_succeeded():
                error_message = response.error_msg().encode('latin-1', errors='replace')
                print(f"Failed to insert vertex {row['标识']} into tag {tag_name}: {error_message}")

def create_edge(session, edge_type):
    try:
        response = session.execute(f"CREATE EDGE IF NOT EXISTS {edge_type}();")
        if not response.is_succeeded():
            print(f"Failed to create edge {edge_type}: {response.error_msg()}")
    except Exception as e:
        print(f"An error occurred while creating edge {edge_type}: {str(e)}")

def insert_tree_edges(session, data: pd.DataFrame):
    last_id_at_level = {}
    for index, row in data.iterrows():
        current_level = row['Indentation Level']
        current_id = row['标识']
        parent_id = None
        if current_level > 1:
            parent_level = current_level - 1
            parent_id = last_id_at_level.get(parent_level)
            if parent_id is not None:
                try:
                    response = session.execute(f"INSERT EDGE {edge_type} () VALUES '{parent_id}' -> '{current_id}' : ();")
                    if not response.is_succeeded():
                        print(f"Failed to create edge from {parent_id} to {current_id}: {response.error_msg()}")
                except Exception as e:
                    print(f"Error creating edge from {parent_id} to {current_id}: {str(e)}")
        last_id_at_level[current_level] = current_id

def insert_bom(session, file_path, space_name, edge_type):
    data = pd.read_csv(file_path, encoding='gbk')
    processed_data = process_file_id(data)
    session.execute_json(f"USE {space_name}")
    tag_insertion(session, processed_data)
    vertex_import_by_type(session, processed_data)
    create_edge(session, edge_type)
    insert_tree_edges(session, processed_data)

config = Config()
connection_pool = ConnectionPool()
ok = connection_pool.init([('10.60.218.13', 9669)], config)
if not ok:
    raise Exception("Failed to initialize connection pool.")
with connection_pool.session_context('root', '@80915108') as session:
    file_path = f"/home/hanrui/Downloads/CA1181P62K1L4E6A21120A_CA1181P62K1L4E6A214X2_E2_Design.csv"
    space_name = f"pythonapitest"
    edge_type = f"include"
    insert_bom(session, file_path, space_name, edge_type)
connection_pool.close()

```