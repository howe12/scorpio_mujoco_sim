from setuptools import setup

package_name = 'seb_naver_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/planning.yaml']),
        ('share/' + package_name + '/launch', ['launch/seb_naver_full.launch.py']),
    ],
    install_requires=['setuptools', 'casadi', 'numpy'],
    zip_safe=True,
    maintainer='seb_naver',
    maintainer_email='dev@seb-naver.org',
    description='SEB-Naver planner + MPC (ROS2 Python port)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner_node = seb_naver_planner.planner_node:main',
            'mpc_node = seb_naver_planner.mpc_node:main',
        ],
    },
)
