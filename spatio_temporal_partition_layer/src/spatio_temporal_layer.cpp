/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "spatio_temporal_partition_layer/spatio_temporal_partition_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/logging.hpp"
#include <cstdint>
#include <exception>
#include <nav2_costmap_2d/cost_values.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <optional>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

using json = nlohmann::json;

namespace rmf
{

void strip_trailing_slash(std::string& str) {
    if (!str.empty() && str.back() == '/') {
        str.pop_back();
    }
}

int extract_robot_id(std::string& name) {
  int robot_id;
  if (sscanf(name.c_str(), "/robot%d/", &robot_id) == 1) {
    // Successfully extracted robot_id
  }
  else {
    throw std::exception();
  }
  return robot_id;
}

SpatioTemporalPartitionLayer::SpatioTemporalPartitionLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
SpatioTemporalPartitionLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  publisher_ = node->create_publisher<geometry_msgs::msg::Point>("next_goal",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  need_recalculation_ = false;
  current_ = true;
  RCLCPP_ERROR(logger_, "Initiallizing grid");
  current_robot_ = node->get_namespace();
  strip_trailing_slash(current_robot_);
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
SpatioTemporalPartitionLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  pose_recvd_ = true;
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}



// Callback function to write curl response to a string
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

std::string get_base_uri()
{
  const char* env = std::getenv("SP_SERVER_BASE");

  if (env == nullptr) {
    return "http://127.0.0.1:3000";
  }

  return env;
}

// Function to perform a CURL POST request with a JSON body
std::string curl_post_request(const std::string& url, const json& jsonData) {
    CURL* curl;
    CURLcode res;
    std::string readBuffer;
    struct curl_slist* headers = NULL;

    std::string jsonStr = jsonData.dump();

    curl = curl_easy_init();
    if (curl) {
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonStr.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);

        if (res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        }
    }
    return readBuffer;
}

std::optional<json> retrieve_currently_allocated_space(const std::string base_server, float x, float y, std::size_t agent_id)
{
    
    std::string url = base_server + "/update_pose";
    
    // Create a JSON object to send
    json postData;
    postData["agent_id"] = agent_id;
    postData["x"] = x;
    postData["y"] = y;
    postData["angle"] = 0.0;

    std::cout << "Making a POST request to " << url << "..." << std::endl;
    std::string response = curl_post_request(url, postData);

    if (!response.empty()) {
        std::cout << "Response from " << url << ":\n" << response << std::endl;
        return json::parse(response);
    } else {
        std::cout << "Response from " << url << " was empty." << std::endl;
        return std::nullopt;
    }
}


// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
SpatioTemporalPartitionLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
SpatioTemporalPartitionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }
  if (!pose_recvd_) {
    RCLCPP_ERROR(logger_, "No pose received");
    return;
  }
  auto allocated_space = retrieve_currently_allocated_space(get_base_uri(), robot_x_, robot_y_, extract_robot_id(current_robot_));
  if (!allocated_space.has_value())
  {
    RCLCPP_ERROR(logger_, "Could not reach server %s", get_base_uri().c_str());
    return;
  }

  geometry_msgs::msg::Point point;
  point.x = allocated_space.value()["next_goal"][0];
  point.y = allocated_space.value()["next_goal"][1];
  publisher_->publish(point);

  auto path = allocated_space.value()["remaining_traj"];
  for (auto tx: path) {
    float ax = tx[0];
    float ay = tx[1];
    RCLCPP_INFO(logger_, "Logger (%f,%f)",  ax, ay);
  }

  std::unordered_set<std::size_t> safe_spots;
  float cell_size = allocated_space.value()["cell_size"];
  RCLCPP_INFO(logger_, "Cell size is %f", cell_size);
  for(auto c: allocated_space.value()["allocated_free_space"])
  {
    float x = c[0];
    float y = c[1];
    float bmin_x = x - cell_size/2;
    float bmin_y = y - cell_size/2;
    float bmax_x = x + cell_size/2;
    float bmax_y = y + cell_size/2;
    
    unsigned int m_min_x,m_min_y, m_max_x, m_max_y;
    master_grid.worldToMap(bmin_x, bmin_y, m_min_x, m_min_y);
    master_grid.worldToMap(bmax_x, bmax_y, m_max_x, m_max_y);       
    for (auto bx = m_min_x; bx <= m_max_x; bx++) {
      for (auto by = m_min_y; by <= m_max_y; by++) {
        auto index = master_grid.getIndex(bx, by);
        safe_spots.insert(index);
      } 
    }
  }
  
  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);
  for (int mi = min_i; mi < max_i; mi++) {
    for (int mj = min_j; mj < max_j; mj++) {
      double wx, wy;
      master_grid.mapToWorld(mi, mj, wx, wy);
      auto id = master_grid.getIndex(mi, mj);
      if (safe_spots.count(id) != 0) {
        continue;
      }
      master_array[id] = LETHAL_OBSTACLE;
    }
  }

}

}  // namespace rmf

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rmf::SpatioTemporalPartitionLayer, nav2_costmap_2d::Layer)
