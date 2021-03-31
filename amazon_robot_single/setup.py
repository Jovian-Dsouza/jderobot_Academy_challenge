from setuptools import setup

package_name = 'amazon_robot_single'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jovain',
    maintainer_email='dsouzajovian123@gmail.com',
    description='JdeRobot Amazon warehouse single robot solution',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amazon_warehouse_single = amazon_robot_single.amazon_warehouse_single:main',
        ],
    },
)
