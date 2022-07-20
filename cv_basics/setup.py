from setuptools import setup
import os
from glob import glob

package_name = 'cv_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'torch'],
    zip_safe=True,
    maintainer='alberto',
    maintainer_email='amigud00@estudiantes.unileon.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_recorder = cv_basics.img_recorder:main',
            'img_analyser = cv_basics.img_analyser:main',
            'img_visualizer = cv_basics.img_visualizer:main',
        ],
    },
)
