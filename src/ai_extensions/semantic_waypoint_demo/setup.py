from glob import glob
from setuptools import find_packages, setup

package_name = "semantic_waypoint_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="siyao",
    maintainer_email="siyao@example.com",
    description="AI extension proof-of-concept that converts semantic detections into autonomy waypoints.",
    license="GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "semantic_waypoint_node = semantic_waypoint_demo.semantic_waypoint_node:main",
        ],
    },
)
