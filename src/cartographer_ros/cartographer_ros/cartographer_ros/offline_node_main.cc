/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/offline_node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "ros/ros.h"

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/TransformStamped.h"
#include <fstream>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

namespace cartographer_ros {

namespace carto = ::cartographer;
void ExportPbstream(const std::string& pbstream_filename, const std::string& out_dir, int id) {
  carto::io::ProtoStreamReader reader(pbstream_filename);
  carto::io::ProtoStreamDeserializer deserializer(&reader);
  carto::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const carto::mapping::proto::Trajectory& trajectory_proto =
      pose_graph_proto.trajectory(0);
  std::string out_file_name = out_dir + std::to_string(id) + ".tjc";
  std::ofstream out_file(out_file_name, std::ios::out);
  for (int i = 0; i < trajectory_proto.node_size(); ++i) {
    const auto& node = trajectory_proto.node(i);
    const auto& universal_stamp = node.timestamp();
    const auto& carto_stamp = carto::common::FromUniversal(universal_stamp);
    ros::Time now = cartographer_ros::ToRos(carto_stamp);
    const auto& pose = node.pose();
    const auto& translation = pose.translation();
    const auto& q = pose.rotation();
    double angle = 2 * atan2(q.z(), q.w());
    if (angle > M_PI)
      angle -= 2 * M_PI;
    else if (angle < -M_PI)
      angle += 2 * M_PI;  
    out_file << now.toNSec() << " " << translation.x() << " " << translation.y() << " " << angle << std::endl;
  }
  out_file.close();
  std::cout << "Trajectory" << 0 << " is written to '" << out_file_name << "'\n";
}
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::ros::init(argc, argv, "cartographer_offline_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  const cartographer_ros::MapBuilderFactory map_builder_factory = [](
      const ::cartographer::mapping::proto::MapBuilderOptions&
          map_builder_options) {
    return ::cartographer::mapping::CreateMapBuilder(map_builder_options);
  };

  std::string bag_filename, pbstream_name;
  bool save_flag = cartographer_ros::RunOfflineNode(map_builder_factory, bag_filename, pbstream_name);
  if (save_flag == false) {
    ::ros::shutdown();
    return 0;
  }
  ROS_INFO("Offline Cartographer finished, prepare to output states...");
  size_t slash_pos = bag_filename.find_last_of("/");
  std::string bag_name = bag_filename.substr(slash_pos);
  size_t dot_pos = bag_name.find_last_of(".");
  bag_name = bag_name.substr(0, dot_pos);
  std::string output_dir = "/home/stn/slam/trajectories" + bag_name + "/";
  std::string output_path = output_dir + "carto_";
  std::string command = "scrot -b " + output_dir + "thumbnails/carto_%Y-%m-%d-%H-%M-%S.png";
  system(command.c_str());
  if (!fs::exists(output_dir))
    fs::create_directory(output_dir);
  cartographer_ros::ExportPbstream(pbstream_name, output_path, cartoTrajOutputCheck(output_dir));
  ::ros::shutdown();
  return 0;
}
