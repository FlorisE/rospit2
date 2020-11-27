# Copyright (c) 2020 AIST.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Setup script for ROSPIT2."""

import setuptools

package_name = 'rospit2'

with open('README.md', 'r') as fh:
    long_description = fh.read()

setuptools.setup(
    name=package_name,
    version='2.0.1',
    author='Floris Erich',
    author_email='floris.erich@aist.go.jp',
    maintainer='Floris Erich',
    maintainer_email='floris.erich@aist.go.jp',
    description='Library for Physical Integration Testing',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/FlorisE/rospit2',
    packages=[package_name],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
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
            'test_script = rospit2.run_tests:main'
        ],
    },
    package_data={package_name: ['xml/rospit.xsd']},
)
