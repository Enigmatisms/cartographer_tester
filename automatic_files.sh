#!/usr/bin/env bash
# cartographer 自动跑包脚本

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/stn/slam/carto/devel_isolated/setup.bash 
cd /home/stn/slam/carto/

path="/home/stn/Dataset/other/"
traj_base="/home/stn/slam/trajectories/"
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
    carto_traj_num=`find ${traj_output_folder}/ -maxdepth 1 -name "carto_*"| wc -l`
    if [ ${carto_traj_num} -gt 10 ]; then
        echo "There are already at least 10 trajectory files, skipping bag: ${bag_name}..."
        continue
    fi
    mkdir -p "${traj_output_folder}/thumbnails/"
    for ((k=0;k<1;k++)); do
        echo "Running bag '${bag_name}', precombine_threshold: ${map_distance}m"
        roslaunch cartographer_ros offline_my_robot.launch bag_filenames:=${file} save_state_filename:="${traj_output_folder}/loc.pbstream" state_filename:="${traj_output_folder}/traj.pbstream"
    done
    echo ${bag_name} >> ${log_file_name}
done