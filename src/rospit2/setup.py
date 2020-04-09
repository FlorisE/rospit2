from setuptools import setup

package_name = 'rospit2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='floris',
    maintainer_email='floris.erich@aist.go.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_script = rospit2.test_script:main'
        ],
    },
    package_data={package_name: ['xml/rospit.xsd']},
)
