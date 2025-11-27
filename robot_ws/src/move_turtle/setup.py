from setuptools import find_packages, setup

package_name = 'move_turtle'

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
    maintainer='yuanying',
    maintainer_email='yuanying@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auto_explore = move_turtle.auto_explore:main',
            'move_circle = move_turtle.move_circle:main',
            'move_full_path = move_turtle.move_full_path:main',
            'test_move_command = move_turtle.test_move_command:main',
        ],
    },
)
