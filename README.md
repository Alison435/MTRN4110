# MTRN4110

Header and source files provided are available on master, files specific to each section/part are in branch
Note: Python files run using Python 2.7 in Windows (not virtual environment)

- exploreTest.py = manual control over robot (Arduino loaded with drive_sensors.ino). Allows for straight, right and left turns to test out exploration and direction/explore decisions (through Bluetooth)
- vision_command.py = sends a command string of ^>^<^^>^\n to the robot (Arduino loaded with drive_sensors.ino). Runs once (Through Bluetooth)
- drive_sensors.ino = Arduino file to use for Exploration testing. Contains code for actuation.
- hardware_definitions.h = driving pin OOP definitions

- PhaseB_Drive_Explore.ino = set up structure for driving with exploration logic (autonomously)
- drive_sensors.ino = driving with exploration logic (manual)
- PhaseB_CV_Planning_Auto.ino = whole pipeline for Floodfill + auto mode (robot acts upon directions)
