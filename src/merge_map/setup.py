import os
from glob import glob

from setuptools import find_packages, setup


package_name = "merge_map"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(
        exclude=["test"],
    ),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.py"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*"),
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Seongil Heo",
    maintainer_email="your-email@example.com",
    description=(
        "TF-aware online occupancy-grid merger "
        "for multiple robots"
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "merge_map = merge_map.merge_map:main",
            (
                "offline_merge_map = "
                "merge_map.offline_merge_map:main"
            ),
        ],
    },
)