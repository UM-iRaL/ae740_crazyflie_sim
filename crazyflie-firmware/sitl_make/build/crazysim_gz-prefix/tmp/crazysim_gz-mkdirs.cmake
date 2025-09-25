# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/../tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim")
  file(MAKE_DIRECTORY "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/../tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim")
endif()
file(MAKE_DIRECTORY
  "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/build/build_crazysim_gz"
  "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/build/crazysim_gz-prefix"
  "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/build/crazysim_gz-prefix/tmp"
  "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/build/crazysim_gz-prefix/src/crazysim_gz-stamp"
  "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/build/crazysim_gz-prefix/src"
  "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/build/crazysim_gz-prefix/src/crazysim_gz-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/build/crazysim_gz-prefix/src/crazysim_gz-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/atharva/Teaching/AE740_Labs/ae740_crazyflie_sim/crazyflie-firmware/sitl_make/build/crazysim_gz-prefix/src/crazysim_gz-stamp${cfgdir}") # cfgdir has leading slash
endif()
