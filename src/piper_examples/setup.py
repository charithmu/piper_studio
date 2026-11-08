from setuptools import find_packages, setup

package_name = "piper_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", ["urdf/carrier.urdf.xacro"]),
        ("share/" + package_name + "/launch", ["launch/carrier_demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Charith Munasinghe",
    maintainer_email="mung@zhaw.ch",
    description="Example: a carrier robot with the Piper mounted on it.",
    license="Apache-2.0",
)
