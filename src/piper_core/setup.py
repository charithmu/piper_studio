from setuptools import find_packages, setup

package_name = "piper_core"

setup(
    name=package_name,
    version="0.4.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["numpy>=1.24"],  # no ROS: plain Python and numpy, usable from any project (pip install src/piper_core)
    python_requires=">=3.10",
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Charith Munasinghe",
    maintainer_email="mung@zhaw.ch",
    description="Piper arm rules without ROS: joint limits, command guard, named poses.",
    license="Apache-2.0",
)
