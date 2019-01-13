# FRC Team 4145's 2019 Robot Code
This repository contains all of the code running on FRC Team 4145's robot controller or RoboRIO.
## Contents
#### Robot_code -- Contains the java code project running on the RoboRIO
#### Other Entries
## Development Environment Setup
In order to setup the development environment a few things must be installed on your computer. There are two available methods. The second method will work with existing setups but requires more work.
#### Method One
WPIlib Provides an installer for Windows(Known working) as well as Mac and linux. The installer is avaliable [here](https://github.com/wpilibsuite/allwpilib/releases) **Note: As of 1/12/2019 the Mac and linux versions are having issues.**
1. Download your version of the installer above.
2. Run the installer. it will bring you to a selection menu. Select the Download Visual Studio Code button. The installer will download VSCode from the Web then take you back to the selection menu. If you want, you may un-check the C++ option. Select run and let the installer install and load all of the components needed for development. It will populate 3 special icons for ShuffleBoard (a dashboard), Visual Studio Code and SmartDashboard.
3. Clone this repository onto the development computer.
4. Select file then select open folder and navigate to the directory the project was cloned into. 
5. Profit??

#### Method Two
1. Download your version of Visual Studio Code avaliable [here](https://code.visualstudio.com/)
2. Download the WPIlib plugin avaliable [here](https://github.com/wpilibsuite/vscode-wpilib/releases)
3. Run the Visual Studio Code installer.
4. Open Visual Studo Code and navigate to extensions. Search for and install the java extension pack. Once complete select the ... above the search bar, then selecting install from vsix. Navigate to your downloads folder and select the WPIlib plugin downloaded earlier.
5. Clone this repository onto the development computer.
6. Select file then select open folder and navigate to the directory the project was cloned into. 
7. Profit??

## Code Explanation
#### src/main/java/frc/robot
This package contains all robot specific code. Subsystems contains its generic template class as well as the other subsystem classes. The actions package contains all robot actions used in the state machine or in operator functionality. **TODO ADD OTHER PACKAGES** 

#### src/main/java/frc/lib
This package contains all helper functions derived previously. Each sub-package's contents are related to its name. 
