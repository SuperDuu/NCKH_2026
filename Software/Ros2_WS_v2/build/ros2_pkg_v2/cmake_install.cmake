# Install script for directory: /home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/install/ros2_pkg_v2")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/environment" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/environment" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros2_pkg-0.0.0-py3.12.egg-info" TYPE DIRECTORY FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_python/ros2_pkg/ros2_pkg.egg-info/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/ros2_pkg" TYPE DIRECTORY FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/install/ros2_pkg_v2/lib/python3.12/site-packages/ros2_pkg"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE EXECUTABLE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/imu_process_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/CMakeFiles/imu_process_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE EXECUTABLE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/humanoid_llc_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/CMakeFiles/humanoid_llc_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v2")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v2"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE EXECUTABLE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/humanoid_llc_node_v2")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v2")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v2"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v2")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/CMakeFiles/humanoid_llc_node_v2.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v2")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v2"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE EXECUTABLE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/imu_process_node_v2")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v2")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v2"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v2")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/CMakeFiles/imu_process_node_v2.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v3" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v3")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v3"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE EXECUTABLE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/humanoid_llc_node_v3")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v3" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v3")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v3"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/humanoid_llc_node_v3")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/CMakeFiles/humanoid_llc_node_v3.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v3" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v3")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v3"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE EXECUTABLE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/imu_process_node_v3")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v3" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v3")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v3"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2/imu_process_node_v3")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/CMakeFiles/imu_process_node_v3.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE PROGRAM RENAME "export_onnx" FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/export_onnx.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE PROGRAM RENAME "visual" FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/visual.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE PROGRAM RENAME "stand_test" FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/stand_test.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE PROGRAM RENAME "reactive_balance" FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/reactive_balance.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE PROGRAM RENAME "ppo_balance_sim_20hz" FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/ppo_balance_sim_20hz.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE PROGRAM RENAME "ppo_balance_sim_20hz_phase2" FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/ppo_balance_sim_20hz_phase2.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros2_pkg_v2" TYPE PROGRAM RENAME "cpg_phase_1" FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/cpg_phase_1.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2" TYPE DIRECTORY FILES
    "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/launch"
    "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/urdf"
    "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/config"
    "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/worlds"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/urdf" TYPE DIRECTORY FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/urdf/meshes")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ros2_pkg_v2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ros2_pkg_v2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/environment" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/environment" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_index/share/ament_index/resource_index/packages/ros2_pkg_v2")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/cmake" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2/cmake" TYPE FILE FILES
    "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_core/ros2_pkg_v2Config.cmake"
    "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/ament_cmake_core/ros2_pkg_v2Config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros2_pkg_v2" TYPE FILE FILES "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/build/ros2_pkg_v2/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
