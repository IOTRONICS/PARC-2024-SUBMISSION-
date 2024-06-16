from setuptools import find_packages, setup

package_name = 'tomatoes_detection1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the weights directory
        (f'share/{package_name}/weights', ['weights/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christian',
    maintainer_email='christian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task2_solution = tomatoes_detection1.task2_solution:main',
        ],
    },
)
