from glob import glob

from setuptools import setup

package_name = 'smart_llm_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['prompts/*.py']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Andrea Basuroski',
    description='SMART-LLM multi-robot task planner for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = smart_llm_planner.planner_node:main',
            'smart_llm = smart_llm_planner.smart_llm:main',
        ],
    },
)
