from setuptools import setup

package_name = 'ntrip'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='honggyu1220',
    maintainer_email='honggyu1220@example.com',
    description='ROS 2 NTRIP client for VRS-based RTK correction',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ntrip_client_node = ntrip.ntrip_client_node:main',
            'rtcm_decoder_node = ntrip.rtcm_decoder_node:main',
            'gga_generator_node = ntrip.gga_generator_node:main',
        ],
    },
)

