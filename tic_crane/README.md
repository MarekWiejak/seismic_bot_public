## Package Summary
The tic_crane package provides ROS interface for Tic Stepper Motor Controller (https://www.pololu.com/category/212/tic-stepper-motor-controllers).

## Prequisites
This package requires "tic" library (https://github.com/pololu/pololu-tic-software). Adjust CMakeLists.txt to fit location of the library in your system.
This package depends on raspi_gpio package.

This implementation assumes using a stepper motor with a worm gear reductor for its self-locking property.

## Node
This package provides tic_crane_node ROS node which act as a server for Tic Stepper Motor Controller services and actions.

## Actions
- ~move_to_target_pose (tic_crane/MoveToPosition): Moves sensor to given postition (in steps of motor relative to "zero" position) with given speed (absolute value in steps of motor per second). Yields current position as feedback. Returns final position as result.
- ~recover (tic_crane/Recover): Recovers sensor from unknown position to "stored" position with given speed (signed value in steps of motor per second) and time limit (in seconds). Returns a boolean value (True if recovered sensor before timeout, False if failed).

## Subscribed Topics
- none

## Published Topics
- none

## Services
- ~reset_position (std_srvs/Empty): Sets current physical position of the sensor as position "zero".

## Parameters
- none
