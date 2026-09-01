"""
RoboMantis 双臂真机 MoveIt Launch
================================
左臂 left_  = 原 geni_craner1 (RS01 J1, motor 8-14, can2/can3, robomantis_hw_rs01 插件)
右臂 right_ = 原 geni_craner  (RS06 J1, motor 1-7,  can0/can1, robomantis_hw_rs06 插件)
base 间距 35cm（左臂 +Y 0.175，右臂 -Y 0.175）。

启动顺序：
  1. ros2_control_node（加载两个不同的 hardware system）
  2. robot_state_publisher
  3. joint_state_broadcaster
  4. left/right joint_trajectory_controller
  5. move_group（双臂规划）
  6. left/right zero_torque_controller（inactive，需手动激活）
  7. rviz2（可选）
"""
import os
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz2"),
    ]
    use_rviz = LaunchConfiguration("use_rviz")

    # ==================== URDF（双臂 xacro，内含两个 hardware 包） ====================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("robomantis_description"),
            "urdf", "robomantis_dual.urdf.xacro"
        ]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ==================== SRDF ====================
    robot_description_semantic_content = Command([
        "cat ",
        PathJoinSubstitution([
            FindPackageShare("robomantis_moveit_config"),
            "config", "robomantis_dual.srdf"
        ]),
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # ==================== MoveIt 参数 ====================
    kinematics_yaml = load_yaml("robomantis_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("robomantis_moveit_config", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}
    ompl_planning_yaml = load_yaml("robomantis_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}
    moveit_controllers_yaml = load_yaml("robomantis_moveit_config", "config/moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ==================== ros2_control 控制器参数（必须以 yaml【文件】形式传入）====================
    # 【零力矩修复·关键】humble 的 controller_manager 中，每个子控制器只能从参数【文件】里
    # 按“控制器节点名 -> ros__parameters”取参数；launch 内联字典（含扁平 dotted-key 覆盖）
    # 只会挂到 controller_manager 顶层节点，永远到不了子控制器。
    #   - 原 launch 用内联 dotted-key 覆盖 urdf_path，零力矩控制器根本读不到，
    #     只拿到 yaml 里的相对文件名 -> Pinocchio buildModel 失败 -> 输出 0 力矩 -> 手臂瘫；
    #   - 若整个改成内联字典，则连 type 都读不到（所有控制器 spawn 失败）。
    # 正确做法：读取包内 yaml -> 在嵌套结构里把两个零力矩控制器的 urdf_path 改成绝对路径
    # （其余参数原样不动）-> 写成一个临时 yaml【文件】-> 把该文件路径传给 ros2_control_node。
    desc_share = get_package_share_directory("robomantis_description")
    ros2_controllers_params = load_yaml("robomantis_moveit_config", "config/ros2_controllers.yaml")
    if ros2_controllers_params is not None:
        ros2_controllers_params["left_zero_torque_controller"]["ros__parameters"]["urdf_path"] = \
            os.path.join(desc_share, "urdf", "geni_craner_rs01_single.urdf")
        ros2_controllers_params["right_zero_torque_controller"]["ros__parameters"]["urdf_path"] = \
            os.path.join(desc_share, "urdf", "geni_craner_rs06_single.urdf")
        resolved_yaml_path = os.path.join(
            tempfile.gettempdir(), "robomantis_ros2_controllers_resolved.yaml")
        with open(resolved_yaml_path, "w") as f:
            yaml.safe_dump(ros2_controllers_params, f, sort_keys=False, allow_unicode=True)
    else:
        # 兜底：读不到包内 yaml 时退回原始文件路径（不会比原方案更差）
        resolved_yaml_path = PathJoinSubstitution([
            FindPackageShare("robomantis_moveit_config"),
            "config", "ros2_controllers.yaml"
        ])

    # ==================== Nodes ====================
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, resolved_yaml_path],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_trajectory_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_joint_trajectory_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )

    # 零力矩控制器默认 inactive，需要时用 ros2 control switch_controllers 激活
    left_zero_torque_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_zero_torque_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60", "--inactive"],
    )
    right_zero_torque_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_zero_torque_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60", "--inactive"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("robomantis_moveit_config"),
            "config", "moveit.rviz"
        ])],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            {"robot_description_kinematics": kinematics_yaml},
        ],
        condition=IfCondition(use_rviz),
    )

    # ==================== 启动顺序 ====================
    delay_left_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )
    delay_right_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )
    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    nodes = [
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_left_controller,
        delay_right_controller,
        delay_move_group,
        left_zero_torque_spawner,
        right_zero_torque_spawner,
        rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes)
