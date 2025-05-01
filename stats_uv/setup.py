from setuptools import find_packages, setup

package_name = 'stats_uv'

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
    maintainer='np',
    maintainer_email='nerpoc42@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uv_dose_tracker = stats_uv.stats_dose:main',
            'uv_time_tracker = stats_uv.stats_time:main',
            'uv_pos_tracker = stats_uv.stats_pos:main',
        ],
    },
)
