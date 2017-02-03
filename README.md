# RobotCasserole2017
Robot Casserole robot source code for the 2017 FRC game, First Steamworks.

## Contents

Here's a high level overview of what we did this year:
- [Path-planned autonomous on a mechanum drivetrain](https://github.com/RobotCasserole1736/MecanumPathPlanner)
- [High goal vision target detection](https://github.com/RobotCasserole1736/RobotCasserole2017/wiki/Vision-Target-Qualification-Algorithm)
- Custom robot website for driver information, debugging information, and value calibration
- PID control of drivetrain velocity, vision alignment, and shooter wheel
- Automated ball launch system with integrated control of shooter, vision, and hopper feed
- Advanced robot performance monitoring and logging

### Robot Code

Eclipse project containing 2017-specific code, Casserole common library code, external jar libraries, and robot website resources.

### BBB

Python source code for our vision target identification algorithm running on a Beaglebone Black coprocessor. More information about the setup can be found on [this wiki page](https://github.com/RobotCasserole1736/RobotCasserole2017/wiki/Vision-Target-Identification-System).

### Calculators

GNU Octave (aka Matlab) simulations of fuel trajectories, used for initial prototype development.

### logFileSnagger

Python scripts to collect log file from the robot over FTP after matches when we're in the pit. Also includes a script used for uploading files to an AWS server once we get internet connection.

### DataViewer2

A javascript/HTML based viewer of data logs captured from the robot during operation.


