from setuptools import setup

package_name = 'interactiva_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/interactiva_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_email@ejemplo.com',
    description='Paquete interactivo para segmentación de mesa con RealSense',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'table_segmentation = interactiva_ros.segmentation:main',
        ],
    },
)
