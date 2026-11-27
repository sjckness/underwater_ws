from setuptools import setup

setup(
    name="link_monitor",
    version="0.1.0",
    packages=["link_monitor"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/link_monitor"]),
                ("share/link_monitor", ["package.xml"])],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["monitor = link_monitor.monitor:main"]},
)
