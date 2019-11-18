# clhero_common

Compile errors:

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
