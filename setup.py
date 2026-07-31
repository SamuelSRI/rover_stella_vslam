from glob import glob
from setuptools import find_packages, setup

package_name = "rover_stella_vslam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Samuel Gendre",
    maintainer_email="samuel@example.com",
    description="Stella-VSLAM integration for the rover localization selector.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "stella_pose_adapter = rover_stella_vslam.stella_pose_adapter:main",
        ],
    },
)
