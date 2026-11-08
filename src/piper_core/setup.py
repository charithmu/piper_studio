from setuptools import find_packages, setup

package_name = "piper_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[],  # no ROS and nothing else: plain Python, usable from any project (pip install src/piper_core)
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Charith Munasinghe",
    maintainer_email="mung@zhaw.ch",
    description="Piper arm rules without ROS: joint limits, command guard, named poses.",
    license="Apache-2.0",
)
