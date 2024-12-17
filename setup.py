from setuptools import setup, find_packages

setup(
    name="extTello",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "djitellopy<=2.5.0",
        "numpy",
        "rclpy",
    ],
    author="Philip Lundström",
    author_email="philip.lundstrom@live.se",
    description="Extension of djitellopy",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/my_package",  # URL for your package
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)

