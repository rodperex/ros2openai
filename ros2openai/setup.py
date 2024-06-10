from setuptools import find_packages, setup

package_name = 'ros2openai'

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
    maintainer='rod',
    maintainer_email='rodrigo.perez@urjc.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'service = gptros2.gpt_service:main',
        'client = gptros2.gpt_client:main',
        ],
    },
)
