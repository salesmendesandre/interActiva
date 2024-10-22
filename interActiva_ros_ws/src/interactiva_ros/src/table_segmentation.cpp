#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

class TableSegmentation : public rclcpp::Node
{
public:
    TableSegmentation()
    : Node("table_segmentation_node")
    {
        // Suscriptor para recibir la nube de puntos
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/realsense_camera/depth/color/points", 1, std::bind(&TableSegmentation::point_cloud_callback, this, std::placeholders::_1));

        
        // Publicador para la nube segmentada (mesa)
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_plane", 1);
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convertir el mensaje PointCloud2 a un objeto PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Configurar la segmentación de plano (RANSAC)
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No planar surface found.");
            return;
        }

        // Extraer los inliers (el plano, que sería la mesa)
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*table_cloud);

        // Convertir de vuelta a PointCloud2
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*table_cloud, output_msg);
        output_msg.header = msg->header;

        // Publicar la nube segmentada
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TableSegmentation>());
    rclcpp::shutdown();
    return 0;
}
