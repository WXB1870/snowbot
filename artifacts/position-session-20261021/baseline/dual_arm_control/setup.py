from setuptools import setup

setup(
    name="snowbot_dual_arm_control",
    version="0.0.0",
    packages=["dual_arm_control"],
    package_dir={"": "scripts"},
    install_requires=["numpy", "scipy"],
    zip_safe=True,
)
