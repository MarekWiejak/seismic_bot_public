## Package Summary
The sensor_server package provides ROS interface for a sensor attached to the crane.

## Node
This package provides sensor_server_node ROS node which provides simulation of seismic measurement accesed via ROS action. The simulation doesn't return any fake measurement data, but does provide verbose of the process. It is desgned to be used only as a placeholder for real measurement software and testing the crane module state machine.

## Actions
- ~measurement (sensor_server/Measurement): Provides simulation of seismic measurement with verbose. Takes in required time of measurement. Yields ammount of measurement time left as feedback and returns status (if the measurement succeded) as result.

## Subscribed Topics
- none

## Published Topics
- none

## Services
- none

## Parameters
- none
