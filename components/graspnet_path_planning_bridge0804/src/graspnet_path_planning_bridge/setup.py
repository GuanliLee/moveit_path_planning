from glob import glob
from setuptools import find_packages, setup

package_name = "graspnet_path_planning_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ligl",
    maintainer_email="ligl@example.com",
    description="Bridge between GraspNet task output and path_planning_service.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge_node = graspnet_path_planning_bridge.bridge_node:main",
        ],
    },
)
