#!/usr/bin/env bash
# cartographer 自动跑包脚本

# Launch the robot
source /opt/ros/melodic/setup.bash 
source /home/wy/slam/carto/devel_isolated/setup.bash 
cd /home/wy/slam/carto/

path="/home/wy/Dataset/hfps/"
traj_base="/home/wy/slam/trajectories/"
all_files=(`find ${path} -maxdepth 1 -type f -name "*.bag"`)
output_file_name=`date +%Y-%m-%d-%H-%M-%S`
log_file_name="${traj_base}${output_file_name}.log"
echo "Start processing..." &> ${log_file_name}
cat file_list.conf | while IFS=' ' read file; do
    file=${path}${file}
    bag_name=${file##*"/"}
    bag_name=${bag_name%.*}
    traj_output_folder="${traj_base}${bag_name}"
    if [ ! -d ${traj_output_folder}/ ]; then
        mkdir -p ${traj_output_folder}
    fi
    carto_traj_num=`find ${traj_output_folder}/ -name "carto_odom_*"| wc -l`
    if [ ${carto_traj_num} -gt 5 ]; then
        echo "There are already at least 10 trajectory files, skipping bag: ${bag_name}..."
        continue
    fi
    mkdir -p "${traj_output_folder}/thumbnails/"
    for ((k=0;k<6;k++)); do
        echo "Running bag '${bag_name}', precombine_threshold: ${map_distance}m"
        roslaunch cartographer_ros offline_my_robot.launch bag_filenames:=${file} save_state_filename:="${traj_output_folder}/traj.pbstream" > /dev/null 2> /dev/null
    done
    echo ${bag_name} >> ${log_file_name}
done