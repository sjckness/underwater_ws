from setuptools import setup

setup(
    name="pose_cmd",
    version="0.1.0",
    packages=["pose_cmd"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/pose_cmd"]),
                ("share/pose_cmd", ["package.xml"])],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["cmd = pose_cmd.cmd:main"]},
)
