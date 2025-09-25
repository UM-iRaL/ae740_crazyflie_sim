include(ExternalProject)
ExternalProject_Add(crazysim_gz
  SOURCE_DIR ${CF2_SITL_ROOT_DIR}/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  BINARY_DIR ${CF2_SITL_BINARY_DIR}/build_crazysim_gz
  INSTALL_COMMAND ""
  USES_TERMINAL_CONFIGURE true
  USES_TERMINAL_BUILD true
  EXCLUDE_FROM_ALL false
  BUILD_ALWAYS 1
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j ${parallel_jobs}
)