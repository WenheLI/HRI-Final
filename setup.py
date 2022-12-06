from setuptools import setup

package_name = 'hide_seek'

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
    maintainer='eric',
    maintainer_email='wl692@cornell.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_movement_node = hide_seek.detect_movement_node:main',
            'countdown_node = hide_seek.countdown:main',
            'calling_node = hide_seek.callingout:main'
        ],
    },
)
