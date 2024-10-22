cmake_minimum_required(VERSION 3.5)
project(interactiva_ros)

# Encontrar las dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # Incluir OpenCV
find_package(cv_bridge REQUIRED)  # Incluir cv_bridge

# Compilar el nodo de segmentación
add_executable(table_segmentation src/table_segmentation.cpp)
ament_target_dependencies(table_segmentation rclcpp sensor_msgs pcl_conversions PCL)

# Compilar el nodo de filtrar objetos sobre la mesa
add_executable(filter_objects_above_table src/filter_objects_above_table.cpp)
ament_target_dependencies(filter_objects_above_table rclcpp sensor_msgs pcl_conversions PCL)

# Compilar el nodo de conversión de nube de puntos a imagen
add_executable(point_cloud_to_image_node src/point_cloud_to_image.cpp)
ament_target_dependencies(point_cloud_to_image_node rclcpp sensor_msgs pcl_conversions PCL cv_bridge)

# Enlazar librerías de OpenCV al nodo de conversión
target_link_libraries(point_cloud_to_image_node ${OpenCV_LIBRARIES})

# Instalar los ejecutables
install(TARGETS
  table_segmentation
  filter_objects_above_table
  point_cloud_to_image_node  # Añadir el nodo de conversión a la instalación
  DESTINATION lib/${PROJECT_NAME})

# Instalar los archivos de lanzamiento
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
  
# Finalizar el paquete
ament_package()
