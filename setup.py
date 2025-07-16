from setuptools import setup, find_packages

package_name = 'robot_kinemic'

setup(
    name=package_name,
    version='0.0.1',
    # packages=[package_name],
    packages=find_packages(),
    include_package_data=True,
    package_data={
        package_name: ["model/urdfs/*.urdf"],
        package_name: ["config/*.ini"],
    },

    data_files=[
        ('share/ament_index/resource_index/packages',
         

            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='magiclib',
    maintainer_email='magiclib@service.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node_robot_kinemic = robot_kinemic.robot_control:main",
            "action_robot_kinemic = robot_kinemic.robot_control_action:main",
        ],
    },
)
