from setuptools import find_packages, setup

package_name = "dent_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adithya",
    maintainer_email="adithya@example.com",
    description="RealSense capture, registration, and dent-mapping ROS 2 nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "realsense_frame_capture_node = "
            "dent_mapping.realsense_frame_capture_node:main",
            "capture_scan_gui_client_example = "
            "dent_mapping.capture_scan_gui_client_example:main",
            "dent_reconstruction_node = dent_mapping.dent_reconstruction_node:main",
            "dent_yolo_node = dent_mapping.dent_yolo_node:main",
            
        ],
    },
)
