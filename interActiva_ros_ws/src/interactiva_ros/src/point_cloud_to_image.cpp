#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

class PointCloudToImage : public rclcpp::Node
{
public:
    PointCloudToImage()
    : Node("point_cloud_to_image_node")
    {
        // Suscriptor para recibir la nube de puntos de las manos
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_above_table", 1, std::bind(&PointCloudToImage::point_cloud_callback, this, std::placeholders::_1));
        
        // Publicador para la imagen en escala de grises
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/interaction_image", 1);
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convertir el mensaje PointCloud2 a un objeto PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Parámetros de la imagen (tamaño 1920x1080 en escala de grises)
        int image_width = 1920;
        int image_height = 1080;
        cv::Mat gray_image = cv::Mat::zeros(image_height, image_width, CV_8UC1);  // Imagen de 8 bits (escala de grises)

        // Definir los límites para la proyección de los puntos 3D en la imagen
        float x_min = -0.5, x_max = 0.5;  // Límites en metros para x
        float y_min = -0.5, y_max = 0.5;  // Límites en metros para y

        // Definir el rango de valores de z
        float z_min = 0.0, z_max = 1.0;  // Ajustar según el rango de z (distancia de la cámara)

        // Recorrer todos los puntos y proyectarlos en la imagen 2D
        for (const auto& point : cloud->points) {
            // Proyectar x e y al rango de la imagen (conviértelo a coordenadas de píxeles)
            int pixel_x = static_cast<int>((point.x - x_min) / (x_max - x_min) * image_width);
            int pixel_y = static_cast<int>((point.y - y_min) / (y_max - y_min) * image_height);

            // Asegurarse de que las coordenadas proyectadas estén dentro de los límites de la imagen
            if (pixel_x >= 0 && pixel_x < image_width && pixel_y >= 0 && pixel_y < image_height) {
                // Mapear el valor de z al rango [0, 255] para la escala de grises
                float z_normalized = (point.z - z_min) / (z_max - z_min);  // Normalizar z entre z_min y z_max
                z_normalized = std::max(0.0f, std::min(1.0f, z_normalized));  // Limitar el valor entre 0 y 1
                int intensity = static_cast<int>(z_normalized * 255);  // Convertir a escala de grises

                // Asignar el valor de intensidad al píxel correspondiente
                gray_image.at<uchar>(pixel_y, pixel_x) = intensity;
            }
        }

        // Convertir la imagen OpenCV a un mensaje ROS para publicarla
        sensor_msgs::msg::Image ros_image;
        std_msgs::msg::Header header;  // Crear un header vacío (ajústalo si es necesario)
        cv_bridge::CvImage(header, "mono8", gray_image).toImageMsg(ros_image);

        // Publicar la imagen
        image_publisher_->publish(ros_image);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToImage>());
    rclcpp::shutdown();
    return 0;
}