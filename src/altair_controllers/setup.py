from setuptools import setup

package_name = 'altair_controllers'
package_list = [
    package_name,
    package_name + '/modules',
    package_name + '/modules/dxl_type'
]

setup(
    name                = package_name,
    version             = '0.0.0',
    packages            = package_list,
    data_files          = [
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
            'app_controller = altair_controllers.app_controller:main'
        ],
    },
)