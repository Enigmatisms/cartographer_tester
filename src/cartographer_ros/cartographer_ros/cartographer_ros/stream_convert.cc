#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/TransformStamped.h"
#include <fstream>

namespace cartographer_ros {

namespace carto = ::cartographer;
void ExportPbstream(const std::string& pbstream_filename, const std::string& outname) {
  carto::io::ProtoStreamReader reader(pbstream_filename);
  carto::io::ProtoStreamDeserializer deserializer(&reader);
  carto::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  
  for (size_t trajectory_id = 0; trajectory_id < pose_graph_proto.trajectory().size();
       ++trajectory_id) {
    const carto::mapping::proto::Trajectory& trajectory_proto =
        pose_graph_proto.trajectory(trajectory_id);
    std::string out_file_name = outname + "_" + std::to_string(trajectory_id) + ".txt";
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
    std::cout << "Trajectory" << trajectory_id << " is written to '" << out_file_name << "'\n";
  }
}
}

int main(int argc, char** argv) {
    if (argc < 3) {
      std::cerr << "Usage: stream_convert <path to .pbstream file> <output file path>\n";
      return -1;
    }
    std::string file_path = std::string(argv[1]), pathname = std::string(argv[2]) + "carto_traj";
    cartographer_ros::ExportPbstream(file_path, pathname);
    return 0;
}