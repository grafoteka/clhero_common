# clhero_common

#Compile errors:

ERROR:-- Could not find the required component 'epos_library'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "epos_library" with
  any of the following names:

    epos_libraryConfig.cmake
    epos_library-config.cmake

  Add the installation prefix of "epos_library" to CMAKE_PREFIX_PATH or set
  "epos_library_DIR" to a directory containing one of the above files.  If
  "epos_library" provides a separate development package or SDK, be sure it
  has been installed.
Call Stack (most recent call first):
  clhero_hw_control/CMakeLists.txt:10 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/jorge/ws/raul_ws/build/CMakeFiles/CMakeOutput.log".
See also "/home/jorge/ws/raul_ws/build/CMakeFiles/CMakeError.log".
Invoking "cmake" failed

SOLUTION: 
Install Epos Library package ->
cd ~/your_ws/src
git clone -b kinetic-devel https://github.com/RIVeR-Lab/epos_hardware.git

# HOW TO RUN A SIMULATION:
You need 4 terminals.

1.- Launch the Gazebo simulation with the robot

    $ roslaunch clhero_gazebo clhero.launch
2.- Launch the controllers. You need to check the launch file:

    <arg name="gazebo_sim" default="true"/>
    <arg name="hardware" default="false"/>

    $ roslaunch clhero_sim_controllers clhero_sim_controllers.launch
3.- Launch the gait controller. It loads the registered gait patterns and the parameters for reconfiguring the gait patterns. By default: position control test, reverse position control test, alternating tripod, turn right tripod, turn left tripod, open loop alternating tripod, stand up, lay down, offset setting

    $ roslaunch clhero_gait_controller gait_controller.launch
    
4.- Run the control terminal: To start moving press (1), to stop (2), to quit (q) 

    $ rosrun clhero_gait_controller control_terminal
    
    ------------------------------------------------------------
	Terminal for clhero gait control 
    ------------------------------------------------------------
    Select a command: 
    [1] Start gait pattern
    [2] Stop gait pattern
    [3] Pause gait pattern execution
    [4] Continue paused execution
    [5] Force gait state
    [6] Update movement arguments
    [h] Show main menu
    [q] Exit
    ------------------------------------
    [Command]: 1
    
    -- Start gait pattern --
    Available patterns: 
    [1] complete_wave
    [2] test_gait_pattern
    [3] position_control_test
    [4] reverse_position_control_test
    [5] alternating_tripod
    [6] turn_right_tripod
    [7] turn_left_tripod
    [8] open_loop_alternating_tripod
    [9] stand_up
    [10] lay_down
    [11] offset_setting
    Introduce the desired pattern to start (press q to cancel):
    
