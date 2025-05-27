import yaml
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Default update rate
    default_update_rate = 100
    update_rate = default_update_rate

    # Get controller configuration
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("caddy_ai2_ros2_control_hardware_curtis_motor_driver"),
            "config",
            "curtis_motor.yaml",
        ]
    )

    # Crear un contexto de lanzamiento
    context = LaunchContext()

    # Evaluar y obtener el valor real de la sustitución
    resolved_controller_config_path = controller_config.perform(context)

    # Imprimir el valor resuelto
    print(f"[DEBUG] path del YAML: {resolved_controller_config_path}")

    # Leer el contenido del archivo YAML
    with open(resolved_controller_config_path, 'r') as yaml_file:
        try:
            # Cargar el contenido del YAML
            yaml_content = yaml.safe_load(yaml_file)
            # Imprimir el contenido del YAML
            print("[DEBUG] YAML Content:", yaml_content)
            # Obtener el valor de update_rate o usar 100 como predeterminado
            update_rate = yaml_content.get("controller_manager", default_update_rate).get("ros__parameters", default_update_rate).get("update_rate", default_update_rate)
        except yaml.YAMLError as exc:
            print(f"[ERROR] Error al cargar el YAML: {exc}")


    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("caddy_ai2_ros2_control_hardware_curtis_motor_driver"), "urdf", "curtis_motor.urdf.xacro"]
            ),
            " ",  # Espacio para separar argumentos
            f"update_rate:={update_rate}",
        ]
    )

    # Evaluar el contenido del comando
    resolved_robot_description_content = robot_description_content.perform(context)

    print(f"[DEBUG] robot_description_content: {resolved_robot_description_content}")

    robot_description = {"robot_description": robot_description_content}

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
        [control_node] + load_controllers
    )