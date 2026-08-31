from setuptools import find_packages, setup


setup(
    name="tianji_teleop",
    version="0.1.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="boen",
    maintainer_email="boen@localhost",
    description="Portable Mocap/H5 to Marvin teleoperation runtime.",
    license="Proprietary",
)
