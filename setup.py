from os import path
from setuptools import setup

with open(path.join(path.dirname(path.abspath(__file__)), "README.md")) as f:
    readme = f.read()

setup(
    name="TLC_sumo",
    version="1.0.0",
    description="Run Smart Traffic Light Controller simulations with SUMO.",
    long_description=readme,
    author="computate",
    author_email="smartabyar-smartvillage@computate.topicbox.com",
    url="https://smartvillage.computate.org",
    packages=["TLC_sumo"],
    install_requires=[],
    test_suite="nose.collector",
    tests_require=["nose"],
    license="MIT",
    zip_safe=False,
    python_requires=">=3.6",
    entry_points={"console_scripts": ["TLC_sumo = TLC_sumo.__main__:main"]},
)
