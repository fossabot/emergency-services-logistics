import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('emergency_services_logistics')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/esl-domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    fill_box_cmd = Node(
        package='emergency_services_logistics',
        executable='fill_box_action_node',
        name='fill_box_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    give_content_cmd = Node(
        package='emergency_services_logistics',
        executable='give_content_action_node',
        name='give_content_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    hold_carrier_cmd = Node(
        package='emergency_services_logistics',
        executable='hold_carrier_action_node',
        name='hold_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    
    load_carrier_cmd = Node(
        package='emergency_services_logistics',
        executable='load_carrier_action_node',
        name='load_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    move_cmd = Node(
        package='emergency_services_logistics',
        executable='move_action_node',
        name='move_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
    move_carrier_cmd = Node(
        package='emergency_services_logistics',
        executable='move_carrier_action_node',
        name='move_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    release_carrier_cmd = Node(
        package='emergency_services_logistics',
        executable='release_carrier_action_node',
        name='release_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    swalo_cmd = Node(
        package='emergency_services_logistics',
        executable='satisfied_with_at_least_one_action_node',
        name='satisfied_with_at_least_one_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
    unfill_box_cmd = Node(
        package='emergency_services_logistics',
        executable='unfill_box_action_node',
        name='unfill_box_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    unload_carrier_cmd = Node(
        package='emergency_services_logistics',
        executable='unload_carrier_action_node',
        name='unload_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
        
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(fill_box_cmd)
    ld.add_action(give_content_cmd)
    ld.add_action(hold_carrier_cmd)
    ld.add_action(load_carrier_cmd)
    ld.add_action(move_cmd)
    ld.add_action(move_carrier_cmd)
    ld.add_action(release_carrier_cmd)
    ld.add_action(swalo_cmd)
    ld.add_action(unfill_box_cmd)
    ld.add_action(unload_carrier_cmd)
    

    return ld
