from setuptools import find_packages, setup

package_name = "piper_core"

setup(
    name=package_name,
    version="0.11.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["numpy>=1.24"],  # no ROS: plain Python and numpy, usable from any project (pip install src/piper_core)
    python_requires=">=3.10",
    extras_require={"test": ["pytest"], "arm": ["pyAgxArm"]},     # `arm`: the real-arm console needs the vendor SDK
    entry_points={"console_scripts": ["piper-arm = piper_core.arm_cli:main"]},
    zip_safe=True,
    maintainer="Charith Munasinghe",
    maintainer_email="mung@zhaw.ch",
    description="Piper arm rules without ROS: joint limits, command guard, named poses.",
    license="Apache-2.0",
)
