import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pcl
from pcl import PointCloud_PointXYZ
import sensor_msgs.point_cloud2 as pc2

class TableSegmentationNode(Node):
    def __init__(self):
        super().__init__('table_segmentation_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.point_cloud_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/segmented_plane', 10)

    def point_cloud_callback(self, msg):
        # Convertir el mensaje PointCloud2 a PCL PointCloud
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        pcl_cloud = pcl.PointCloud_PointXYZ()
        pcl_cloud.from_list(cloud_points)

        # Segmentación del plano con RANSAC
        seg = pcl_cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)

        indices, coefficients = seg.segment()

        # Extraer inliers (el plano que asumimos es la mesa)
        table_cloud = pcl_cloud.extract(indices, negative=False)

        # Convertir de vuelta a PointCloud2 y publicar
        table_msg = self.convert_pcl_to_ros(table_cloud)
        self.publisher.publish(table_msg)

    def convert_pcl_to_ros(self, pcl_array):
        ros_msg = PointCloud2()
        # Lógica de conversión de PCL a PointCloud2
        return ros_msg

def main(args=None):
    rclpy.init(args=args)
    node = TableSegmentationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
