import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # <--- 3. ADD THIS LINE BELOW
        # This copies everything from your 'launch' folder to the install directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sahniel',
    maintainer_email='sahniel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_turtle = my_turtle_controller.move_turtle:main',
            'smart_turtle = my_turtle_controller.smart_turtle:main',
            'tester = my_turtle_controller.tester:main',
            'tester_2 = my_turtle_controller.tester_2:main',

        ],
    },
)
