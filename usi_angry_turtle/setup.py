from setuptools import setup
import glob

package_name = 'usi_angry_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/usi_angry_turtle/launch', ['launch/launch_draw_usi.py']),
        ('share/usi_angry_turtle/data', glob.glob('data/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linus',
    maintainer_email='43851079+kurma99@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move2goal = usi_angry_turtle.move_2_goal_node:main',
            'state_manager = usi_angry_turtle.state_manager_node:main',
            'enemy_turtle = usi_angry_turtle.enemy_turtle_mode:main',
        ],
    },
)
