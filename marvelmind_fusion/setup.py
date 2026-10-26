from setuptools import setup, find_packages

package_name = 'marvelmind_fusion'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'fusion_node = marvelmind_fusion.fusion_node:main',
            'goto_goal = marvelmind_fusion.goto_goal:main',
            'perimeter = marvelmind_fusion.perimeter:main',
            'calibrate_yaw = marvelmind_fusion.calibrate_yaw:main',
        ],
    },
)
