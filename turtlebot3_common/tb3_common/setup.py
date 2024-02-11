from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tb3_common'

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))

def package_files(directory, data_files):
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            data_files.append(("share/" + package_name + "/" + path, glob(path + "/**/*.*", recursive=True)))
    return data_files

# Add directories
data_files = package_files("launch/", data_files)
data_files = package_files("rviz/", data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nakalab',
    maintainer_email='nakalab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joy_teleop = tb3_common.teleop_joy:main",
            "cam_teleop = tb3_common.teleop_cam:main",
        ],
    },
)
