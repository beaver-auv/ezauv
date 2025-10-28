from setuptools import setup, find_packages

setup(
    name='ezauv',
    version='0.1.9',
    packages=find_packages(),
    install_requires=[
        'gurobipy==12.0.1',
        'numpy==2.2.3',
        'pygame==2.6.1',
        'scipy==1.15.2',
        'imageio[ffmpeg]'
        ],
    description='A library to make coding AUVs easier',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    author='Andre Gordon',
    author_email='gordona26@bcdschool.org',
    url='https://github.com/beaver-auv/ezauv',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License'
    ],
)
