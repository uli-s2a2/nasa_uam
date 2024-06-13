from setuptools import find_packages, setup

package_name = 'actor_critic_mpc_controller'

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
    maintainer='llanesc',
    maintainer_email='christian.llanes@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actor_critic_mpc_controller = actor_critic_mpc_controller.main:main'
        ],
    },
)
