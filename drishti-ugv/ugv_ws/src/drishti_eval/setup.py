from setuptools import find_packages, setup

package_name = 'drishti_eval'

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
    description='Evaluation harness for DRISHTI-UGV: localisation metrics, '
                'stop-latency measurement and fault injection.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'evaluate_trajectory = drishti_eval.evaluate_trajectory:main',
            'fault_injector = drishti_eval.fault_injector_node:main',
            'plan_suite = drishti_eval.plan_suite:main',
        ],
    },
)
