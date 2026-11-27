from setuptools import setup

setup(
    name="dvl_dr",
    version="0.1.0",
    packages=["dvl_dr"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/dvl_dr"]),
                ("share/dvl_dr", ["package.xml"]),
                ("share/dvl_dr/config", ["config/ekf.yaml"]),
                ("share/dvl_dr/launch", ["launch/dvl_dr.launch.py"])],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["preprocess = dvl_dr.preprocess:main"]},
)
