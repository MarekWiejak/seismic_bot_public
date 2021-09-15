## Package Summary
The raspi_gpio package provides a ROS interface for Raspberry GPIO. Available functionalities are: checking current state of a binary input; observing state of a binary input until a signal edge is detected. All functionalities are advertised as ROS Services or ROS Actions.

## Prequisites
This package requires RPi.GPIO python library.

Current implementation assumes usage of pull-up digital inputs on Rasapberry Pi.

## Node
This package provides gpio_server_node ROS node which act as a server for Raspberry Pi GPIO interface services and actions.

# Actions
- ~main_upper_ls_monitor_states (raspi_gpio/WaitForEdge): Waits for signal egde detection. As input takes mode of detection (True for rising edge, False for falling edge). As feedback returnes initial state of the pin. As result returnes state of the pin after edge detection. 
- ~aux_upper_ls_monitor_states (raspi_gpio/WaitForEdge): Waits for signal egde detection. As input takes mode of detection (True for rising edge, False for falling edge). As feedback returnes initial state of the pin. As result returnes state of the pin after edge detection. 

# Subscribed Topics
- none

# Published Topics
- none

# Services
- ~main_upper_ls_check_state (raspi_gpio/EmptyBool): Takes no input and returnes current state of an binnary input connected to main limit switch.
- ~aux_upper_ls_check_state (raspi_gpio/EmptyBool): Takes no input and returnes current state of an binnary input connected to auxiliary limit switch.

# Parameters
- ~limit_switch_main_id (int): ID of a Raspberry GPIO pin connected to main limit switch of the crane module.
- ~limit_switch_aux_id (int): ID of a Raspberry GPIO pin connected to auxiliary limit switch of the crane module.

