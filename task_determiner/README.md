## Package summary
The seismic_bot package provides an algorithm determining next task (navigation, measurement, etc.) when asked by robot_state_machine.

## Node
The fake_task_generator node provides a siumlator of decisive algorithm with three hardcoded position points indicating to which the robot should move and take measurement. The poses should be changed whenever the map changes.

## Actions
- none

## Subscribed Topics
- none

## Published Topics
- none

## Services
- new_task (task_determiner/NewTask): polls for new task, returns information what should be next task, whether the measurement should be taken after movementm, the coordinates of next measurement point
- fake_task_generator_init (std_srvs/Empty): restart list of remaining positions of required measurement

## Parameters
- none
