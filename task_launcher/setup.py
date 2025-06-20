from setuptools import find_packages, setup

package_name = 'task_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task_launch.launch.py']),
        ('share/' + package_name + '/trees', ['trees/nav2_custom_tree.xml']),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='themis',
    maintainer_email='ef2000th@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
