from setuptools import find_packages, setup

package_name = 'task_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/ui', ['task_gui/ui/main_window.ui', 'task_gui/ui/start_window.ui']),
        (f'share/{package_name}/ui/images', ['task_gui/ui/images/TerraCrop.png', 'task_gui/ui/images/tu_delft.png', 'task_gui/ui/images/MDPInc.png', 'task_gui/ui/images/map_new.png']),
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
            'gui_node = task_gui.gui_node:main',
        ],
    },
)
