# RepRapFirmware_robot
only cpp, h and binaries

based on RRF 3.5.0beta3. To compile, use all related projects which are tagged with this release number.

Changes needed for compilation:
- Config/Pins.h add a SUPPORT_ROBOT flag and set to 1
- set other types to 0, for 6HC Hangprinter flag must be set (used for CAN)
- Kinematics.h change robot5axis to robot
- Kinematics.cpp add robot case to initiate the robot Kinematics class
- include RobotKinematics.* into Movement/Kinematics folder

The changed Pins.h, Kinematics.h and .cpp are checked in.

Duet 2 compilation is critical with memory usage, disable all other kinematics in Pins.h to have a chance.

For 6HC, the Hangprinter flag in Pins.h must be set.
