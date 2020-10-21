import setuptools

package_name = 'rospit2'

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name=package_name,
    version="2.0.1",
    author="Floris Erich",
    author_email="floris.erich@aist.go.jp",
    maintainer="Floris Erich",
    maintainer_email="floris.erich@aist.go.jp",
    description="Library for Physical Integration Testing",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/FlorisE/rospit2",
    packages=[package_name],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    python_requires='>=3.6',
    install_requires=['setuptools'],
    zip_safe=True,
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_script = rospit2.test_script:main'
        ],
    },
    package_data={package_name: ['xml/rospit.xsd']},
)
