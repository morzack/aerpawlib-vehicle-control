from setuptools import setup

setup(
        name="aerpawlib",
        version="0.0.1",
        description="Tools and frameworks for writing AERPAW scripts",
        author="John Kesler",
        author_email="jckesle2@ncsu.edu",
        packages=["aerpawlib"],
        install_requires=[
            "dronekit"
            ]
        )
