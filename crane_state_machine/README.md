## Package Summary
The crane_state_machine package provides full realization of measurement procedure, inluding sensor deployment, taking measurement, sensor storing and a number of safety features. The implementation is based on a state machine wrapped into a ROS action.

## Prequisites
This package requires packages:
- raspi_gpio
- tic_crane
- sensor_server

## Node
Crane_state_machine_node contains a SMACH state machine wrapped into a ROS action.

## Actions
- ~crane_state_machine_node (crane_state_machine/CraneStateMachine): Runs complete measurement procedure, takes no inputs and provides no feedback nor result.

## Subscribed Topics
- none

## Published Topics
- none

## Services
- none

## Parameters
- deploy_speed (int): motor speed during sensor deployment (in steps per second)
- deploy_position (int): position of deployed sensor (in motor steps relative to "zero" position)
- store_speed (int): motor speed during sensor storing (in steps per second)
- store_position (int): position of stored sensor (in motor steps relative to "zero" position)
- recover_speed (int): motor speed during sensor recovery (in steps per second)
- recover_timeout (int): time limit for recovery procedure (in seconds)
- fake_measurement_time (int): time of simulated measurement
- measurement_action_name (string): name of measurement action
