# Use CMake to generate visual studio C++ projects
> Prerequisites The visual studio IDE and CMake Tool must be installed on the PC.

## Method 1: Command line mode
- Open the powershell terminal in the 'sample/windows' directory and run the following command to create a C++ project in the' Visual Studio 15 2017 Win64 'environment as an example.
```ps1
mkdir build

cd build

cmake -G "Visual Studio 15 2017 Win64" .. /
` ` `