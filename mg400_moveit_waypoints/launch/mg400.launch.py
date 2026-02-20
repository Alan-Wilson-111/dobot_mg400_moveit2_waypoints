import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        content = yaml.safe_load(f)
        return content if content is not None else {}

def generate_launch_description():
    pkg_dir = get_package_share_directory('mg400_moveit_waypoints')
    
    # Load URDF
    urdf_file = os.path.join(pkg_dir, 'config', 'mg400_moveit.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Load SRDF
    srdf_file = os.path.join(pkg_dir, 'config', 'mg400.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()
    
    # Load config files
    kinematics_yaml = load_yaml('mg400_moveit_waypoints', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('mg400_moveit_waypoints', 'config/joint_limits.yaml')
    
    rviz_config = os.path.join(pkg_dir, 'config', 'moveit.rviz')
    
    # Planning pipeline configuration with Ruckig smoothing
    planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddRuckigTrajectorySmoothing default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        },
    }
    
    # Move group parameters
    move_group_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
    }
    move_group_params.update(planning_pipeline_config)
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        
        # Move Group
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[move_group_params],
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'robot_description_kinematics': kinematics_yaml},
            ],
        ),
    ])