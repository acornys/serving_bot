from setuptools import find_packages, setup

package_name = 'robot5'

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
    maintainer='yoo',
    maintainer_email='yoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
	entry_points={
        'console_scripts': [
            'robot5_manager = robot5.robot5_manager:main',
            'order_ui = robot5.order_ui:main',
        ],
    },
)
