import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Configuration des arguments de ligne de commande
    data_file_arg = DeclareLaunchArgument(
        'file',
        default_value='/home/abdallah/Bureau/PointCloud/data/Lille_1.ply',
        description='Chemin complet vers le fichier .ply'
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel',
        default_value='0.3',
        description='Taille du filtre Voxel (plus grand = plus rapide)'
    )

    # Nœud principal avec vos paramètres et variables
    city_node = Node(
        package='city_analyzer_ros',
        executable='city_node',
        name='city_analyzer_node',
        parameters=[{
            'file_path': LaunchConfiguration('file'),
            'voxel_leaf_size': LaunchConfiguration('voxel'),
            'normal_radius': 0.3,
            'ground_distance_threshold': 0.20
        }], 
        output='screen'
    )

    # Lancement de rviz2 vierge (sans argument -d)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        data_file_arg,
        voxel_size_arg,
        city_node,
        rviz_node
    ])