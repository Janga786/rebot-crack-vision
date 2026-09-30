from glob import glob

from setuptools import find_packages, setup

package_name = "crackvision_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
        (f"share/{package_name}/srdf", glob("srdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="crackvision maintainers",
    maintainer_email="jangarabliss@gmail.com",
    description=(
        "B601-DM end-of-arm overlay (tool tip, wrist D405 + mount collision proxies) "
        "driven by config/robot/end_effector.yaml (ADR-014)."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
