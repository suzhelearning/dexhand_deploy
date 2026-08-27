from setuptools import find_packages, setup


setup(
    name="pico_body_tianji",
    version="0.1.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="boen",
    maintainer_email="boen@localhost",
    description="Portable PICO to Marvin real teleoperation runtime.",
    license="Proprietary",
)
