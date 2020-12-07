# Quick setup for automated code testing and coverage report (WIP)

## Github Actions
[Github Action](https://docs.github.com/en/free-pro-team@latest/actions) is a continuous integration tool built into github itself. It is organized in worklows, in the *.github/workflows* folder. A workflow is basically a list of instruction to give to github's virtual machine. They are organized into jobs, each job being run separately. 
The syntaxt is:
`jobs:`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`name-of-job-1:`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`...`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`name-of-job-2:`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`...`

The syntaxt for running commands in the vm's console is :

`run: |`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`command-1`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`command-2`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`ect`

For the instructions inside the jobs, refer to the *Example* workflow:

There are 3 main parts you need to modify in order to adapt this workflow to your code: 
1. *Dependecies*: External libraries used by your code and not on linux by default, usually installed with the same console command you'd use to install it on your own machine
2. *Scripts commands*: Check that your code runs locally and produce a correct coverage report (as described in the coverage sections). Then, you should just have to replace the commands in the example workflow with your own for it to work. Don't forget to put the commands to enter the correct folder, as the vm starts from the root of the repository.
3. *Upload coverage to codecov*: The `file:` line should point to the coverage file your code produce. Again, see the coverage sections for that.

Sometimes .yml files can be really sensitive to tabulation and spaces; if you encounter such problem, you can try using an online checker for .yml format.
You can check on your pull request the status of the vm's during and after the execution. 
In the example, it is set to run all jobs when a commit is pushed; you can change that at the top of the file if you want.

Finally, be mindful that **there is a limit of minutes per months for these vm's**. As we have an open source repository, this limit is currently at 2000 minutes per month; this gives us plenty of margin, but if you push many times for small changes instead of grouping them into one push, it might drain the time limit. Note that it isn't dramatic if we run out; we just can't automatically test until the next month rolls in.

## Python coverage
For python, we are using coverage.py for code coverage. [You can find a quick tutorial and the full documentation here](https://coverage.readthedocs.io/en/coverage-5.3/), but the gist of it is that you can write your test with either pytest, unittest or nosetest as you would normally, and just switch out the script launch command. The extra specificity for our project is to run the command `coverage xml` in github action to generate a coverage report to send to codecov. You can add an option to this last command to exclude folders from the coverage, like in the example project: `coverage xml --omit="*/test*"` to exclude the *test* folder. The coverage file generated is in the folder you ran the command from. Locally, you can also run the command `coverage report` or `coverage html` to check on your coverage without pushing.
Here is an example python project available for reference.

## C++ setup and coverage
For c++, the setup is a bit more complicated; [Cmake](https://cmake.org) is used for creating MakeFiles automatically to compile the code, [googletest](https://github.com/google/googletest) is used for testing, and [lcov](https://wiki.documentfoundation.org/Development/Lcov) for coverage; As it is more difficult to work with than python, I advise you to copy the example c++ project and adapt it to your code.

Here is how this example project works:

Inside this project you will find: 
- The *cmake* folder; it contains several cmake scripts to generate coverage reports and download the googletest library. You should not have to touch any of those scripts.
- The *src* folder, where the source code is.
- The *tests* folder, where the tests scripts are.

Cmake works on a folder-by-folder basis; inside each folder you need a *CMakeLists.txt* file to tell it what it must do: Inside the *src* folder you need to tell it the different dependencies the files have between them, while inside the *tests* folder you need to add the tests to be run, the files they test, and the custom command to prepare everything for lcov, defined in the *cmake/CodeCoverage* file.

To run the tests, you need to execute these commands:

`mkdir build` 

`cd build`

`cmake .. -DCMAKE_BUILD_TYPE=Debug ..  -Dtest=ON` This command creates all the required makefiles to compile everything, and it also downloads the needed googltest libraries.

`make` To compile everything.

From there you have two choices: you can run `./bin/tests` to run the test without coverage, or you can run `make coverage_tests` to run them with coverage. You can then open `./tests_coverage_report/index.html` to have a detailed view in your browser. The file to be sent to codecov is *./tests_coverage_report.info.cleaned*.

## Codecov
Codecov is a tool that recieves the coverage reports sent by Github Actions, and summarizes them nicely. It display info in the pull request, and you can access more in depth reports on the page associated with the repository.

## Codeclimate
Codeclimate analyses the code in the repository, and will report if it found anything; it checks for things like code duplication, too complexe/long functions, ect. It can sometimes be to aggressive, and might hampers more than it helps; if it is the case, it can be adjusted/issues can be ignored in it's control panel.
