from setuptools import setup

setup(
    name="dvl_relay",
    version="0.1.0",
    packages=["dvl_relay"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/dvl_relay"]),
                ("share/dvl_relay", ["package.xml"])],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["relay = dvl_relay.relay:main"]},
)
