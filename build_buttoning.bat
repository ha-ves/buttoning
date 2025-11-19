colcon --log-level debug build --merge-install --symlink-install ^
  --base-paths src --packages-up-to buttoning_system --cmake-args %ENV_CMAKE_ARGS% ^
  -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF -DENABLE_TESTS=OFF -DWITH_TESTS=OFF ^
  -DCMAKE_CONFIGURATION_TYPES=RelWithDebInfo -DCMAKE_BUILD_TYPE=RelWithDebInfo ^
  -DPython3_FIND_VIRTUALENV=ONLY
