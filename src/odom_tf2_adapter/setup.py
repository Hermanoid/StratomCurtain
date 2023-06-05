from setuptools import setup

package_name = 'odom_tf2_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description="Simple adapter script which republishes the position information from an odometry topic as a tf2 transformation. Used as part of our direct-from-simulation localization 'override' to test the safety curtain",
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf2_adapter = odom_tf2_adapter.OdomTf2Adapter:main'
        ],
    },
)
