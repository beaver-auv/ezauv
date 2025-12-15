from setuptools import setup, find_packages

setup(
    name="ezauv",
    version="0.2.0",
    packages=find_packages(),
    install_requires=[
        "gurobipy",
        "numpy",
        "pygame",
        "scipy",
        "imageio[ffmpeg]",
        "opencv-python",
    ],
    description="A library to make coding AUVs easier",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="Andre Gordon",
    author_email="gordona26@bcdschool.org",
    url="https://github.com/beaver-auv/ezauv",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
    ],
)
