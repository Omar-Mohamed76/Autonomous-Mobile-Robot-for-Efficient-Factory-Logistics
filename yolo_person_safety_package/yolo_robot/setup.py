from setuptools import find_packages, setup


package_name = "yolo_robot"


setup(
    name=package_name,
    version="0.0.0",

    packages=find_packages(
        exclude=[
            "test",
            "tests",
        ]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/" + package_name
            ],
        ),
        (
            "share/" + package_name,
            [
                "package.xml"
            ],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",

    description=(
        "ROS 2 YOLO person detection and "
        "MoveRobot safety action-client package."
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "person_path_detector = "
                "yolo_robot.person_path_detector_node:main"
            ),
            (
                "person_safety_action_client = "
                "yolo_robot.person_safety_action_client_node:main"
            ),
        ],
    },
)
