from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare arguments
    bms_available = DeclareLaunchArgument(
        "bms_available", default_value="true", description="Availability of BMS"
    )
    can_interface = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
    )

    # Nodes
    heros_node = Node(
        package="innok_heros_driver",
        executable="innok_heros_can_driver",
        name="heros_node",
        parameters=[{"can_interface": LaunchConfiguration("can_interface")}],
        remappings=[("battery_state", "battery_state_roctr")],
        respawn=True,
    )

    heros_diagnostics = Node(
        package="innok_heros_driver",
        executable="heros_diagnostics",
        parameters=[{"can_interface": LaunchConfiguration("can_interface")}],
        respawn=True,
    )

    can_bms_node = Node(
        package="innok_heros_driver",
        executable="can_bms_node",
        name="can_bms_node",
        parameters=[{"can_interface": LaunchConfiguration("can_interface")}],
        condition=IfCondition(LaunchConfiguration("bms_available")),
        respawn=True,
    )

    battery_watchdog = Node(
        package="innok_heros_driver",
        executable="battery_watchdog",
        name="battery_watchdog",
        parameters=[
            (
                {"shutdown_behaviour": "battery"}
                if LaunchConfiguration("bms_available")
                else {"shutdown_behaviour": "pc"}
            )
        ],
        respawn=True,
    )

    return LaunchDescription(
        [
            bms_available,
            can_interface,
            heros_node,
            heros_diagnostics,
            can_bms_node,
            battery_watchdog,
        ]
    )
