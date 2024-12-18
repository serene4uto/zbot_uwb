from setuptools import find_packages, setup

package_name = 'zbot_uwb_pydemo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='nguyenhatrung411@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'uwb_follower = zbot_uwb_pydemo.uwb_follower:main',
            # 'nav_to_pose = zbot_uwb_pydemo.nav_to_pose:main',
            # 'odom2uwbref_tf_check = zbot_uwb_pydemo.odom2uwbref_tf_check:main',
            'uwb_waypoints_follower = zbot_uwb_pydemo.uwb_waypoints_follower:main',
        ],
    },
)
