from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can_motor_drv",
            description="CAN interface to use",
        )
    )

    # Initialize Arguments
    can_interface = LaunchConfiguration("can_interface")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("caddy_ai2_description"), "urdf", "caddy_ai2.urdf.xacro"]
            ),
            " ",
            "can_interface:=", can_interface, " "
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Get controller configuration
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("caddy_ai2_ros2_control_hardware_curtis_motor_driver"),
            "config",
            "curtis_motor_driver.yaml",
        ]
    )

    # ROS2 Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
    )

    # Load controllers
    load_controllers = []
    for controller in ["curtis_motor_velocity_controller", "curtis_motor_state_broadcaster"]:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # Return launch description
    return LaunchDescription(
        declared_arguments + [control_node] + load_controllers
    )