from glob import glob

from setuptools import find_packages, setup

package_name = "crackvision_motion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="crackvision maintainers",
    maintainer_email="jangarabliss@gmail.com",
    description=(
        "crackvision ROS 2 overlay: headless MoveIt mock planning against the "
        "B601-DM model (MOT-02)."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plan_joint_goal = crackvision_motion.plan_joint_goal:main",
            "recommend_placement = crackvision_motion.recommend_placement:main",
            "reachability_sweep = crackvision_motion.reachability_sweep:main",
            "apply_scene = crackvision_motion.scene_apply:main",
            "assert_scene_objects = crackvision_motion.assert_scene_objects:main",
            "check_end_effector = crackvision_motion.check_end_effector:main",
        ],
    },
)
