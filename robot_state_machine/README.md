## Package Summary
The robot_state_machine package is an implementation of a state machine defining high level robot behaviour, like managing navigation and measurement tasks.

## Prequisites
This package requires packages:
- raspi_gpio
- tic_crane

## Node
Robot_state_machine_node contains a SMACH state machine.

## Actions
- none

## Subscribed Topics
- none

## Published Topics
- none

## Services
- none

## Parameters
- recover_speed (int): motor speed during sensor recovery (in steps per second)
- recover_timeout (int): time limit for recovery procedure (in seconds)
- new_task_service_name (string): name of a service providing new task for the robot
