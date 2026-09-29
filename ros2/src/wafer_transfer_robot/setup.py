from glob import glob

from setuptools import find_packages, setup


package_name = "wafer_transfer_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name]
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project author",
    maintainer_email="maintainer@example.com",
    description="R-theta-Z wafer-transfer robot simulation and control",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"robot_sim_node = {package_name}.robot_sim_node:main",
            f"controller_node = {package_name}.controller_node:main",
            f"safety_node = {package_name}.safety_node:main",
            f"sequence_node = {package_name}.sequence_node:main",
            f"kinematics_node = {package_name}.kinematics_node:main",
        ],
    },
)
