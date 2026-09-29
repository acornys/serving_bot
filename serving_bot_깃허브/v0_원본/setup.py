from setuptools import find_packages, setup

package_name = 'serving_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team7',
    maintainer_email='team7@todo.todo',
    description='7team serving bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manager = serving_bot.robot5_manager:main',
            'order_ui = serving_bot.order_ui:main',
        ],
    },
)