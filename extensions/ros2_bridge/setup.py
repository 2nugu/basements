from setuptools import setup

package_name = 'basements_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.force_estimators'],
    install_requires=[
        'setuptools',
        'pymycobot>=3.5,<4.0',  # see docs/notes/l3_pipeline_design.md §Q2
        'numpy',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Basements maintainers',
    maintainer_email='hgl@kangwon.ac.kr',
    description='Basements MPM-rigid coupling bridge for ROS 2 (myCobot reference).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'log_trajectory   = basements_ros2.trajectory_logger:main',
            'replay_in_sim    = basements_ros2.trajectory_replayer:main',
            'verify_driver    = basements_ros2.driver_wrapper:verify_main',
        ],
    },
)
