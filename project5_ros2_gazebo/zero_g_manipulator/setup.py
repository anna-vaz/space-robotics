from glob import glob

from setuptools import setup

package_name = "zero_g_manipulator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anna-vaz",
    maintainer_email="avazaiou02@gmail.com",
    description="Free-floating space manipulator in zero-g Gazebo with ros2_control",
    license="MIT",
    entry_points={
        "console_scripts": [
            "step1_torque_pulse = zero_g_manipulator.step1_torque_pulse:main",
            "step2_quintic_move = zero_g_manipulator.step2_quintic_move:main",
            "rns_controller = zero_g_manipulator.rns_controller:main",
            "capture_controller = zero_g_manipulator.capture_controller:main",
        ],
    },
)
