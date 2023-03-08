from setuptools import setup
from setuptools import find_packages

package_name = 'actions'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riboha',
    maintainer_email='riboha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_action = actions.send_action:main',
            'foot_action = actions.foot_action:main',
            'stt_action = actions.stt_action:main',
        ],
    },
)
