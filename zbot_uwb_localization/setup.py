from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zbot_uwb_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('param/*.yaml')),
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
            # 'navuwb_transform_node = zbot_uwb_localization.navuwb_transform_node:main',
            'navuwb_transform_node = zbot_uwb_localization.navuwb_transform_node_opt:main',
        ],
    },
)
