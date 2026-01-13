#include "rclcpp/rclcpp.hpp"
#include "rmf_prototype_msgs/msg/mapf_result.hpp"
#include "rmf_prototype_msgs/msg/mapf_trajectory.hpp"
#include "rmf_prototype_msgs/msg/safe_zone.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cstddef>

using namespace std::placeholders;

/// Agent State holder
class AgentState {
  float x,y;
  std::size_t trajectory_progress;
};

/// Agent controller
class AgentController {

  public: AgentController(rclcpp::Node::SharedPtr node, const std::string& robot_name) {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
      qos.reliable();
      qos.transient_local();
      global_pose_ = node->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>
          (robot_name+"/amcl_pose", 
          qos,
          std::bind(&AgentController::on_pose_update,
                    this, _1));
      node_ptr_ = node;
  }

  void on_pose_update(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr ptr) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Received pose");
  }
  
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_pose_;
  rclcpp::Node::SharedPtr node_ptr_;
  AgentState state_;
};


/// Spatio Temporal Allocation main node.
class SpatioTemporalAllocatorNode: public rclcpp::Node {
  public: SpatioTemporalAllocatorNode(): Node("spatio_temporal_server"), recved_mapf_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "registration", 10, std::bind(&SpatioTemporalAllocatorNode::robot_heartbeat, this, _1));
  }

  private: void robot_heartbeat(std_msgs::msg::String::SharedPtr msg) {
    auto agent_ptr = agents_.find(msg->data);
    if (agent_ptr != agents_.end())
    {
      return;
    }
    if (msg->data == "") {
      RCLCPP_ERROR(this->get_logger(), "Empty message received cant register robot.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Recieved new registration %s", msg->data.c_str());

    AgentController agent(shared_from_this(), msg->data);
    agents_.emplace(msg->data, agent);
  }

  /// Callback for mapf result
  private: void mapf_result(rmf_prototype_msgs::msg::MAPFResult::ConstSharedPtr ptr) {
    if (recved_mapf_)
    {
      return;
    }
    for (auto t: ptr->trajectories)
    {
      auto a = agents_.find(t.robot);
      if (a == agents_.end())
      {
        continue;
      }
    }  
  }

  private: bool recved_mapf_;
  private: rmf_prototype_msgs::msg::MAPFResult::ConstSharedPtr mapf_result_;
  private: std::unordered_map<std::string, AgentController> agents_;
  private: rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpatioTemporalAllocatorNode>());
  rclcpp::shutdown();
}
