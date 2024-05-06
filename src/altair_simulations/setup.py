import os
from setuptools import setup

RESOURCE_PATH   = os.getcwd() + '/resource'
WORLDS_PATH     = os.getcwd() + '/worlds'
RESOURCE_FILES  = [f for f in os.listdir(RESOURCE_PATH) if os.path.isfile(os.path.join(RESOURCE_PATH, f))]
WORLDS_FILES    = [f for f in os.listdir(WORLDS_PATH) if os.path.isfile(os.path.join(WORLDS_PATH, f))]

package_name = 'altair_simulations'
package_list = [
    package_name,
    package_name + '/modules'
]

ext_data_files  = []
for resource_file in RESOURCE_FILES:
    ext_data_files.append(
        ('share/' + package_name + '/resource', ['resource/' + resource_file])
    )
for worlds_file in WORLDS_FILES:
    ext_data_files.append(
        ('share/' + package_name + '/worlds', ['worlds/' + worlds_file])
    )

setup(
    name                = package_name,
    version             = '0.0.0',
    packages            = package_list,
    data_files          = ext_data_files + [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires    = ['setuptools'],
    zip_safe            = True,
    maintainer          = 'dhonan',
    maintainer_email    = 'dhonan.hibatullah@gmail.com',
    description         = 'None',
    license             = 'None',
    tests_require       = ['pytest'],
    entry_points        = {
        'console_scripts': [
            'webots_op3_driver = altair_simulations.webots_op3_driver:main'
        ],
    },
)