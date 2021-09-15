## Package summary
The seismic_bringup package contains all launch and configuration files for seismic metapackage.

## Launch files
Provided launch files may inculde other launch files. Please inspect their content before launching independently.
- amcl: launches localization algorithm
- crane_module: launches crane module functionalities
- gmapping: launches mapping algorithm
- move_base: launches navigation algorithm 
- seismic_basic: launches basic robot functions and crane module
- seismic_main: launches main state machine of the robot defining advanced behaviour

## Configuration files
- crane_sm_parmas: parameters of crane module state machine
- gpio_params: parameters of Raspberry gpio server
- robot_sm_params: parameters of main state machine of the robot
- move base: folder with parameters of navigation
