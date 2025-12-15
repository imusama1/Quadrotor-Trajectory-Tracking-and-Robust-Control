from setuptools import find_packages, setup

package_name = 'arsmc_controller'

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
    maintainer='usama',
    maintainer_email='88314064+imusama1@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'baseline_test = HFTTC_controller.baseline_test:main',
            'check_msg = HFTTC_controller.check_msg:main',
            'HFTTC_controller = HFTTC_controller.HFTTC_controller:main',
            'check_attitude_msg = HFTTC_controller.check_attitude_msg:main',
            'check_local_pos_msg = HFTTC_controller.check_local_pos_msg:main',
            'px4_tf_broadcaster = HFTTC_controller.tf_broadcaster:main',

        ],
    },
)
