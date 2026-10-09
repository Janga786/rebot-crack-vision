from setuptools import find_packages, setup

package_name = "crackvision_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="crackvision maintainers",
    maintainer_email="jangarabliss@gmail.com",
    description=(
        "crackvision ROS 2 camera overlay: one-shot optical-TF and joint-state "
        "capture tools (CAM-05.2)."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "capture_optical_tf = crackvision_camera.capture_optical_tf:main",
            "capture_joint_state = crackvision_camera.capture_joint_state:main",
        ],
    },
)
