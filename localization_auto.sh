#!/usr/bin/env bash
# cartographer localization 自动跑包脚本

# Launch the robot
source /opt/ros/noetic/setup.bash 
source /home/stn/slam/carto/devel_isolated/setup.bash 
cd /home/stn/slam/carto/

path="/home/stn/Dataset/slam_loc/"
traj_base="/home/stn/slam/trajectories/localization_experiment/"
all_files=(`find ${path} -maxdepth 1 -type d -name "hfps*"`)
output_file_name=`date +%Y-%m-%d-%H-%M-%S`
log_file_name="${traj_base}${output_file_name}.log"
echo "Start processing..." &> ${log_file_name}
for file in ${all_files[@]}; do
    bag_name=${file##*"/"}
    # echo ${bag_name}
    traj_output_folder="${traj_base}${bag_name}"
    if [ ! -d ${traj_output_folder}/ ]; then
        mkdir -p ${traj_output_folder}
    fi
    carto_traj_num=`find ${traj_output_folder}/ -name "carto_loc*"| wc -l`
    if [ ${carto_traj_num} -gt 5 ]; then
        echo "There are already at least 5 trajectory files, skipping bag: ${bag_name}..."
        continue
    fi
    mkdir -p "${traj_output_folder}/thumbnails/"
    for ((k=0;k<5;k++)); do
        echo "Running bag '${bag_name}'"
        roslaunch cartographer_ros localization.launch \
            bag_filenames:="${file}/data.bag" \
            save_state_filename:="${traj_output_folder}/loc.pbstream" \
            state_filename:="${file}/traj.pbstream" \
            traj_filename:="${traj_output_folder}/carto_loc" \
            traj_id:=${carto_traj_num} \
             > /dev/null 2> /dev/null
        carto_traj_num=`find ${traj_output_folder}/ -name "carto_loc*"| wc -l`
    done
    echo ${bag_name} >> ${log_file_name}
done