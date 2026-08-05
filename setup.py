from setuptools import find_packages, setup

package_name = 'deimos_science'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trevor',
    maintainer_email='trs0024@mix.wvu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_interface = deimos_science.dynamixel_interface:main',
            'spectrometer = deimos_science.spectrometer_node:main',
            'gimbal_control = deimos_science.gimbal_control:main',
            'spectro_viz = deimos_science.spectrometer_viz:main',
            'spectro_calibrate = deimos_science.spectrometer_calibrate:main'
        ],
    },
)
