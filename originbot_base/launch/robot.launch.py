"""
机器人启动文件
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # 串口设备名称参数 - 用于连接机器人的串口设备
    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyS1',
                                          description='usb bus name, e.g. ttyS1')

    # X轴速度校正因子 - 用于校准机器人前进/后退速度
    correct_factor_vx_arg = DeclareLaunchArgument('correct_factor_vx', default_value='0.898',
                                                  description='correct factor vx, e.g. 0.9')

    # 角速度校正因子 - 用于校准机器人旋转速度
    correct_factor_vth_arg = DeclareLaunchArgument('correct_factor_vth', default_value='0.874',
                                                   description='correct factor vth, e.g. 0.9')

    # 自动停止功能开关 - 当没有接收到控制命令时自动停止机器人
    auto_stop_on_arg = DeclareLaunchArgument('auto_stop_on', default_value='true',
                                             description='auto stop if no cmd received, true or true')

    # IMU传感器使用开关 - 是否启用IMU传感器进行导航
    use_imu_arg = DeclareLaunchArgument('use_imu', default_value='false',
                                        description='if has imu sensor to drive')

    # 里程计发布开关 - 是否发布里程计到base_footprint的TF变换
    pub_odom_arg = DeclareLaunchArgument('pub_odom', default_value='true',
                                         description='publish odom to base_footprint tf, true or true')

    # OriginBot 基础控制节点 - 机器人的核心控制逻辑
    originbot_base_node = Node(
        package='originbot_base',
        executable='originbot_base',
        output='screen',
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'correct_factor_vx': LaunchConfiguration('correct_factor_vx'),
            'correct_factor_vth': LaunchConfiguration('correct_factor_vth'),
            'auto_stop_on': LaunchConfiguration('auto_stop_on'),
            'use_imu': LaunchConfiguration('use_imu'),
            'pub_odom': LaunchConfiguration('pub_odom'),
        }]
    )

    # 静态TF变换发布器 - 从base_footprint到base_link的坐标变换
    # Z轴偏移0.05325米（约5.3厘米），表示机器人底盘到主体结构的高度
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.05325", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "base_footprint", "--child-frame-id", "base_link"]
    )

    # 静态TF变换发布器 - 从base_link到imu_link的坐标变换
    # 无偏移，表示IMU传感器安装在机器人主体上
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "base_link", "--child-frame-id", "imu_link"]
    )

    # 返回启动描述，包含所有参数和节点
    # 启动顺序：参数声明 -> 基础控制节点 -> TF变换发布器
    return LaunchDescription([
        port_name_arg,
        correct_factor_vx_arg,
        correct_factor_vth_arg,
        auto_stop_on_arg,
        use_imu_arg,
        pub_odom_arg,
        originbot_base_node,
        base_footprint_tf,
        imu_tf
    ])