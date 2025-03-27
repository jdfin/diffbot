
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # FindPackageShare("diffbot") -> <ws>/install/diffbot/share/diffbot

    robot_description_content = Command(
        [
            # /opt/ros/jazzy/bin/xacro
            PathJoinSubstitution(
                [FindExecutable(name="xacro")]
            ),
            " ",
            # <ws>/install/diffbot/share/diffbot/description/diffbot_urdf.xacro
            PathJoinSubstitution(
                [FindPackageShare("diffbot"), "description", "diffbot_urdf.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # <ws>/install/diffbot/share/diffbot/config/diffbot_controllers.yaml
    # contains ros parameters for the nodes:
    #   controller_manager
    #   diffbot_base_controller
    robot_controllers = PathJoinSubstitution(
        [ FindPackageShare("diffbot"), "config", "diffbot_controllers.yaml", ]
    )

    # <ws>/install/diffbot/share/diffbot/config/diffbot_controllers.yaml
    # has a section "controller_manager"
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both", # ??? what is this
        #arguments=['--ros-args', '--log-level', 'debug']
    )

    # ??? what is this urdf supposed to contain?
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        #arguments=['--ros-args', '--log-level', 'debug']
    )

    # <ws>/install/diffbot/share/diffbot/config/diffbot_controllers.yaml
    # has a section "diffbot_base_controller"
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args", # following arguments have to all be in the same string
            "--remap /diffbot_base_controller/cmd_vel:=/cmd_vel --remap /diffbot_base_controller/odom:=/odom",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(nodes)
