#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pgo_with_visual/pogolm_api.hpp>

class ClickLandmarkNode : public rclcpp::Node {
public:
  explicit ClickLandmarkNode(std::shared_ptr<pogolm_np::POGOLM> pogolm)
  : Node("click_landmark_node"), pogolm_(std::move(pogolm))
  {
    operating_frame_id_ = "odom";
    assoc_range_ = 0.40f;
    assoc_eps_   = 0.05f;

    clicked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&ClickLandmarkNode::on_clicked_point, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "Ready. RViz Fixed Frame must be '%s'.", operating_frame_id_.c_str());
  }

private:
  void on_clicked_point(const geometry_msgs::msg::PointStamped& msg)
  {
    if (msg.header.frame_id != operating_frame_id_) {
      RCLCPP_WARN(get_logger(),
        "clicked_point frame is '%s' but expected '%s'. Set RViz Fixed Frame to '%s'.",
        msg.header.frame_id.c_str(), operating_frame_id_.c_str(), operating_frame_id_.c_str());
      return;
    }

    // clicked point in world/odom
    const auto& Pw = msg.point;
    const gtsam::Point3 P_w(Pw.x, Pw.y, Pw.z);

    // use PoseGraph's current pose (biased) via POGOLM API
    const gtsam::Pose3 T_wb = pogolm_->get_current_pose();   // WORLD<-BASE (biased)
    const gtsam::Point3 p_local_gtsam = T_wb.inverse().transformFrom(P_w);

    pogolm_np::Point3d p_local(p_local_gtsam.x(), p_local_gtsam.y(), p_local_gtsam.z());

    // Query neighbors
    std::vector<uint64_t> assoc_keys;
    {
      auto neighbors = pogolm_->has_neighbors(p_local, assoc_range_, assoc_eps_);
      assoc_keys.reserve(neighbors.size());
      for (const auto& lm : neighbors) {
        if (!lm) continue;
        assoc_keys.push_back(lm->key);
      }
    }

    const std::string label = "CLICK_" + std::to_string(click_count_++);

    if (assoc_keys.empty()) {
      RCLCPP_INFO(get_logger(),
        "Clicked (%.3f, %.3f, %.3f) -> store %s (no associations)",
        Pw.x, Pw.y, Pw.z, label.c_str());
    } else {
        // if neighbors exist then verify data association
    }

    // store landmarks with thei associated neighbors
    auto new_keys = pogolm_->store_landmark(p_local, label, assoc_keys);
    if (!new_keys.empty()) {
      RCLCPP_INFO(get_logger(), "Inserted landmark id: %llu",
                  (unsigned long long)new_keys[0]);
    }
  }

  std::shared_ptr<pogolm_np::POGOLM> pogolm_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;

  std::string operating_frame_id_;
  float assoc_range_{0.4f};
  float assoc_eps_{0.05f};
  uint64_t click_count_{0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  pogolm_np::APIParams params;
  params.pg_params.lidar_in = "/velodyne_points";
  params.pg_params.odom_in  = "/odom";
  params.pg_params.operating_frame_id = "odom";

  auto pogolm = std::make_shared<pogolm_np::POGOLM>(params);
  auto node   = std::make_shared<ClickLandmarkNode>(pogolm);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_node(node);
  pogolm->attach_to_exec(exec);

  exec->spin();
  rclcpp::shutdown();
  return 0;
}