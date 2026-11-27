from glob import glob

from setuptools import setup

setup(
    name="rov_bringup",
    version="0.1.0",
    packages=["rov_bringup"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/rov_bringup"]),
                ("share/rov_bringup", ["package.xml"]),
                ("share/rov_bringup/launch", glob("launch/*.launch.py")),
                ("share/rov_bringup/config", glob("config/*.yaml")),
                ("share/rov_bringup/profiles", glob("profiles/*.yaml"))],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["record = rov_bringup.record:main"]},
)
