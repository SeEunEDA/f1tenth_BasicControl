from setuptools import setup

package_name = 'twist_to_ackermann_pid'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/twist_to_ackermann_pid.yaml']),
        ('share/' + package_name + '/launch', ['launch/twist_to_ackermann_pid.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vkrtp2',
    maintainer_email='vkrtp2@gmail.com',
    description='Twist->Ackermann with PID and limits',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # launch의 executable 과 일치
            'twist_to_ackermann_pid = twist_to_ackermann_pid.twist_to_ackermann_pid:main',
        ],
    },
)

