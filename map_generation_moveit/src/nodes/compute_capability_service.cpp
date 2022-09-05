#include <ros/ros.h>
#include <map_generation/map_generation.h>
#include <map_generation/ComputeCapability.h>

bool compute(map_generation::ComputeCapability::Request &req, 
             map_generation::ComputeCapability::Response &res)
{
  std::string group_name_ = req.move_group.data;
  double resolution_ = req.sphere_offset.data;
  bool check_collision_ = req.check_collision.data;
  ros::NodeHandle nh_("~");
//   nh_.getParam("group_name", group_name_);
//   nh_.getParam("resolution", resolution_);
//   nh_.getParam("check_collision", check_collision_);
  
  reuleaux::mapGeneration mg(nh_, group_name_, resolution_, check_collision_);
  
  map_generation::WorkSpace ws;
  mg.computeCustomPoints(req.points, ws);
  
  res.ws = ws;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_capability_service");
  ros::NodeHandle nh_("~");

  ros::ServiceServer service = nh_.advertiseService("compute_capability", compute);
  ROS_INFO ("ready to compute capability results");

  ros::spin();
  return 0;
}
