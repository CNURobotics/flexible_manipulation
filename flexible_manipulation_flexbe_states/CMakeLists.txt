cmake_minimum_required(VERSION 3.0.2)
project(flexible_manipulation_flexbe_states)

find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# specify catkin-specific information
# INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
# LIBRARIES - The exported libraries from the project
# CATKIN_DEPENDS - Other catkin projects that this project depends on
# DEPENDS - Non-catkin CMake projects that this project depends on
# CFG_EXTRAS - Additional configuration options
catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME})

# install scripts
install(DIRECTORY src  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
