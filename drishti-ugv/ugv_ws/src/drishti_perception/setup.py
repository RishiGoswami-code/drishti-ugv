from setuptools import find_packages, setup

package_name = 'drishti_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='The Vikings',
    maintainer_email='team@thevikings.example',
    description='Semantic perception and perception health for DRISHTI-UGV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception = drishti_perception.perception_node:main',
        ],
    },
)
