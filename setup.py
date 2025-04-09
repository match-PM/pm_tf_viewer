from setuptools import find_packages, setup

package_name = 'pm_tf_viewer'
submodules = 'pm_tf_viewer/submodules'
setup(
    name=package_name,
    version='0.0.0',
    packages=(package_name, submodules),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mll',
    maintainer_email='terei@match.uni-hannover.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pm_tf_viewer = pm_tf_viewer.pm_tf_viewer:main'
        ],
    },
)
