from setuptools import find_packages, setup

package_name = "piper_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Charith Munasinghe",
    maintainer_email="mung@zhaw.ch",
    description="Python API and CLI for the Piper arm.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "piper = piper_py.cli:main",
        "fake_driver = piper_py.fake_driver:main",
    ]},
)
