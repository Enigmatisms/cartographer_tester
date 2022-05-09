#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <vector>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_listen");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    ros::Rate rate(1000);
    uint64_t last_time = 0;
    std::string traj_filename   = nh.param<std::string>("/tf_listen/traj_filename", "");
    int traj_id                 = nh.param<int>("/tf_listen/traj_id", 0);
    std::string out_file_name   = traj_filename + "_" + std::to_string(traj_id) + ".tjc";
    std::fstream file;
    file.open(out_file_name, std::ios::out);
    while (ros::ok()) {
        rate.sleep();
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/scan",  
                                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.05).sleep();
            continue;
        }
        uint64_t now_time = transform.stamp_.toNSec(), stamp_comp = now_time / 1e6;
        if (stamp_comp == last_time)
            continue;
        last_time = stamp_comp;
        double angle = 2 * atan2(transform.getRotation().z(), transform.getRotation().w());
        file << now_time << " " << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << angle << std::endl;
    }
    file.close();
    printf("Localization trajectory output to '%s'\n", out_file_name.c_str());
    return 0;
}