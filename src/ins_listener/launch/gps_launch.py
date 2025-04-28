import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # 确保从 launch_ros.actions 导入 Node 类

def generate_launch_description():
    return LaunchDescription([
        # 启动发布器节点
        Node(
            package='ins_listener',  # 发布器节点所在的包名
            executable='gps_publisher_from_file',  # 发布器节点的可执行文件
            name='gps_publisher',  # 节点名称
            output='screen',  # 输出到终端
            parameters=[{'param_name': 'value'}],  # 如果有需要的参数，按需添加
        ),

        # 启动订阅器节点
        Node(
            package='ins_listener',  # 订阅器节点所在的包名
            executable='gps_subscriber',  # 订阅器节点的可执行文件
            name='gps_subscriber',  # 节点名称
            output='screen',  # 输出到终端
        ),

        # 启动转换器节点
        Node(
            package='ins_listener',  # 转换器节点所在的包名
            executable='gps_converter_node',  # 转换器节点的可执行文件
            name='gps_converter',  # 节点名称
            output='screen',  # 输出到终端
        ),

        # 启动 RViz2 可视化工具
        Node(
            package='rviz2',  # 使用 rviz2 包
            executable='rviz2',  # rviz2 可执行文件
            name='rviz2',  # 节点名称
            output='screen',  # 输出到终端
            arguments=['-d', '/path/to/your/config.rviz']  # 如果有 RViz 配置文件，指定路径
        ),
    ])



