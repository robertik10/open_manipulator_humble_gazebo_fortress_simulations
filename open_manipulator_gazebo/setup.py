from setuptools import setup

package_name = 'open_manipulator_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'src.end_effector_pos_publisher',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='-',
    maintainer_email='-',
    description='A package for publishing end effector positions',
    license='Apache License 2.0',

    entry_points={
        'console_scripts': [
            'end_effector_pos_publisher = src.end_effector_pos_publisher:main'
        ],
    },
)
