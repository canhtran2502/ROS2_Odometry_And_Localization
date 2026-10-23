import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bumperbot_mujoco"


def data_tree(source, target):
    """Install a directory tree (e.g. meshes/) keeping its structure under share/<package>."""
    entries = {}
    for root, _, files in os.walk(source):
        if files:
            destination = os.path.join("share", package_name, target, os.path.relpath(root, source))
            entries[destination] = [os.path.join(root, name) for name in files]
    return list(entries.items())


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "mjcf"), glob("mjcf/*.xml")),
    ] + data_tree("meshes", "meshes"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Antonio Brandi",
    maintainer_email="antonio.brandi@outlook.it",
    description="MuJoCo simulation of Bumperbot",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_sim_node = bumperbot_mujoco.mujoco_sim_node:main",
        ],
    },
)
