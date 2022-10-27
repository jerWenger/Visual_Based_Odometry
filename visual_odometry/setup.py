from setuptools import setup

package_name = 'visual_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/bringup.py']),
        ('share/' + package_name, ['Calibration Data/ost.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='webmeister',
    maintainer_email='jwenger@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_visualization = visual_odometry.apriltag_visualization:main'
        ],
    },
)
