catkin_make_isolated --install --use-ninja
sed -i "s?use_imu_data = true?use_imu_data = false?g" ./install_isolated/share/cartographer/configuration_files/trajectory_builder_2d.lua