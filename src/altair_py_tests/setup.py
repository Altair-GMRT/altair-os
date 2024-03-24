from setuptools import setup

package_name = 'altair_py_tests'
package_list = [
    package_name,
    package_name + '/modules'
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
            'rclnode_test = altair_py_tests.rclnode_test:main',
            'custom_msg_test = altair_py_tests.custom_msg_test:main'
        ],
    },
)