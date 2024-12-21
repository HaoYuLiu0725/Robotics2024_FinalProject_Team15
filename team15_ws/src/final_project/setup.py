# from setuptools import setup
from setuptools import setup, find_packages

package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='yaotinghuang89@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'final_project_main = final_project.final_project_main:main',
        ],
    },
)
