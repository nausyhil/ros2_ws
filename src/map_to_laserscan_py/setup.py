from setuptools import setup

package_name = 'map_to_laserscan_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='Description of your package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_to_laserscan = map_to_laserscan_py.map_to_laserscan:main'
        ],
    },
)