from setuptools import setup

setup(
    name="yolo_detector",
    version="0.1.0",
    packages=["yolo_detector"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/yolo_detector"]),
                ("share/yolo_detector", ["package.xml", "requirements.txt"])],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["detector = yolo_detector.detector:main"]},
)
