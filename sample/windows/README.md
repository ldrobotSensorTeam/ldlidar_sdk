# Use CMake to generate visual studio C++ projects
> Prerequisites The visual studio IDE and CMake Tool must be installed on the PC.

## Method 1: Command line mode
- Open the powershell terminal in the 'sample/windows' directory and run the following command to create a C++ project in the' Visual Studio 15 2017 Win64 'environment as an example.
``` powershell
mkdir build

cd build

cmake -G "Visual Studio 15 2017 Win64" .. /
```

## Method 2： CMake  GUI

- Create a `build` folder in the `sample/windows` director

	- ![](./pic/1.png)

- Run CMake GUI, complete related configuration, generate VC++ project, compile and run
	- <img src="./pic/2.png" style="zoom:80%;" />

	- <img src="./pic/3.png" style="zoom:80%;" />

	- <img src="./pic/4.png" style="zoom:80%;" />

	- <img src="./pic/5.png" style="zoom:80%;" />

	- <img src="./pic/6.png" style="zoom:80%;" />