from setuptools import find_packages, setup

package_name = "rm75_edu"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="RM75 교육용 예제 모음",
    license="MIT",
    entry_points={
        "console_scripts": [
            "move_home_target = rm75_edu.demos.move_home_target:main",
            "gripper_demo = rm75_edu.demos.gripper_demo:main",
            "mission_scene = rm75_edu.demos.mission_scene:main",
            "pick_place_template = rm75_edu.demos.pick_place_template:main",
        ],
    },
)
