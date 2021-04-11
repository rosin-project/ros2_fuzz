import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="ros2_automatic_fuzzer-jnxf",
    version="0.0.1",
    author="Francisco Martínez Lasaca, Zhoulai Fu, Andrzej Wąsowski",
    author_email="frml@itu.dk, zhfu@itu.dk, wasowski@itu.dk",
    description="An automatic ROS 2 topic fuzzer",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="TODO",
    packages=setuptools.find_packages(),
    install_requires=["Jinja2", "yamale"],
    zip_safe=False,  # TODO: remove
    entry_points={
        "console_scripts": [
            "auto_detector=auto_detector.__main__:main",
            "ros2_fuzzer=ros2_fuzzer.__main__:main",
            "topic_fuzzer=topic_fuzzer.__main__:main",
            "service_fuzzer=service_fuzzer.__main__:main",
            "parameters_fuzzer=parameters_fuzzer.__main__:main",
        ],
    },
    package_data={"": ["*.cpp"]},
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
