from setuptools import find_packages, setup
from glob import glob
import os

package_name = "dianshu_base"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),  # 包含所有种类launch的写法
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dianshu_base = dianshu_base.base_driver:main",
            "fake_base = dianshu_base.fake_base:main",
            "cmd_vel_relay = dianshu_base.cmd_vel_relay:main",
        ],
    },
)
