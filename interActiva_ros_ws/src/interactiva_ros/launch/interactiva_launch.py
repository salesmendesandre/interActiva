from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo para la cámara RealSense
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            parameters=[
                {'pointcloud.enable': True},
                {'depth_module.profile': '1920x1080x30'},   # Resolución de 640x480 a 60 FPS
                {'rgb_camera.enable': True},               # Habilitar la cámara RGB
                {'rgb_camera.profile': '640*480x30'},     # Resolución de 640x480 a 60 FPS
                {'enable_infra1': True},                  # Habilitar cámara infrarroja
                {'enable_infra2': True},                  # Habilitar segunda cámara infrarroja
            ],
        ),
        # Nodo de segmentación de la mesa
        Node(
            package='interactiva_ros',
            executable='table_segmentation',
            name='table_segmentation_node',
            output='screen'
        ),
        Node(
            package='interactiva_ros',
            executable='filter_objects_above_table',
            name='object_filter_node',
            output='screen'
        ),
         # Nodo que convierte los puntos de interacción a una imagen en escala de grises
        Node(
            package='interactiva_ros',
            executable='point_cloud_to_image_node',
            name='point_cloud_to_image',
            output='screen'
        )
    ])