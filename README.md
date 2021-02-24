# EPFL-RT Driverless Section

Github repository for the Driverless Section

Languages and add-ons used by the examples projects:

Python:
- Python 3.x (latest minor release)
- For tests: the latest version of either [pytest](https://docs.pytest.org/en/stable/), [unittest](https://docs.python.org/3/library/unittest.html) or [nosetest](https://nose.readthedocs.io/en/latest/), choose the one you prefer
- For test coverage statistics: the latest version of [Coverage.py](https://coverage.readthedocs.io/en/coverage-5.3/)

C++:
- C++ should be installed by default on your machine, otherwise install the latest one
- For compiling: the latest version of [Cmake](https://cmake.org)
- For tests: the latest version of [googletest](https://github.com/google/googletest), which should install itself if you have configured the cmake files correctly (cf C++ExampleProject)
- For test coverage statistics: the latest version of [lcov](https://wiki.documentfoundation.org/Development/Lcov)

The virtual machine at Github Actions uses the latest version of ubuntu with all the mentionned add-ons.

[Here is a quick guide on using git](QuickGitGuide.md)

[Here is the description of the setup used for automated code testing and coverage report](SetupGuide.md)
