#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <omp.h>  // Para paralelización con OpenMP

class TableSegmentation : public rclcpp::Node
{
public:
    TableSegmentation()
    : Node("table_segmentation_node"), plane_found_(false)
    {
        // Suscriptor para recibir la nube de puntos desde la cámara RealSense
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/realsense_camera/depth/color/points", 1, std::bind(&TableSegmentation::point_cloud_callback, this, std::placeholders::_1));
        
        // Publicador para los puntos filtrados sobre la mesa
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_above_table", 1);
    }

private:
    bool plane_found_;  // Bandera para saber si ya se ha encontrado el plano
    pcl::ModelCoefficients::Ptr coefficients_;  // Coeficientes del plano de la mesa

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convertir el mensaje PointCloud2 a un objeto PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Aplicar un filtro de distancia (PassThrough) para limitar la nube en el eje Z (altura)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");  // Filtrar en el eje Z (altura)
        pass.setFilterLimits(0.0, 1.3);  // Mantener puntos entre 0 y 1.3 metros de altura
        pass.filter(*cloud);

        // Si aún no hemos encontrado el plano, usar RANSAC para segmentarlo
        if (!plane_found_) {
            coefficients_ = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);  // Ajusta según la precisión deseada

            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients_);

            if (inliers->indices.size() == 0) {
                RCLCPP_WARN(this->get_logger(), "No planar surface found.");
                return;
            }

            // Verificar que el plano está a menos de 1.3 metros de la cámara
            float avg_distance = 0.0;
            for (const auto& idx : inliers->indices) {
                avg_distance += cloud->points[idx].z;
            }
            avg_distance /= inliers->indices.size();

            if (avg_distance > 1.3) {
                RCLCPP_WARN(this->get_logger(), "El plano encontrado está a más de 1.3 metros. Se descartará.");
                return;
            }

            plane_found_ = true;  // Marcar que el plano ha sido encontrado
            RCLCPP_INFO(this->get_logger(), "Plano de la mesa encontrado y almacenado.");
        }

        // Optimización del filtrado de puntos sobre el plano
        pcl::PointCloud<pcl::PointXYZ>::Ptr objects_above_table(new pcl::PointCloud<pcl::PointXYZ>);
        objects_above_table->points.reserve(cloud->points.size());  // Reservar memoria

        // Vectorizar los coeficientes del plano
        const float a = coefficients_->values[0];
        const float b = coefficients_->values[1];
        const float c = coefficients_->values[2];
        const float d = coefficients_->values[3];

        // Filtrar puntos en paralelo utilizando OpenMP
        #pragma omp parallel for
        for (int i = 0; i < cloud->points.size(); ++i) {
            const auto& point = cloud->points[i];
            float distance_from_plane = a * point.x + b * point.y + c * point.z + d;

            if (distance_from_plane < -0.01 && distance_from_plane > -0.10) {
                #pragma omp critical
                {
                    objects_above_table->points.push_back(point);
                }
            }
        }

        // Publicar solo los puntos filtrados sobre la mesa
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*objects_above_table, output_msg);
        output_msg.header = msg->header;
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
