import setuptools

setuptools.setup(
    name="dronehover",
    version="0.0.1",
    description="A drone hovering optimizer",
    author="Elijah Ang",
    author_email="e.h.w.ang@tudelft.nl",
    packages=setuptools.find_packages(),
    install_requires=["numpy", "scipy"],
)