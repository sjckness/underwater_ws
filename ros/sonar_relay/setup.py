from setuptools import setup

setup(
    name="sonar_relay",
    version="0.1.0",
    packages=["sonar_relay"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/sonar_relay"]),
                ("share/sonar_relay", ["package.xml"])],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["relay = sonar_relay.relay:main"]},
)
