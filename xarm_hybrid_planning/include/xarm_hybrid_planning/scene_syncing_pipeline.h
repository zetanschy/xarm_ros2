// Global planner para hybrid planning que se sincroniza la escena antes de cada
// plan. Ver el porque en scene_syncing_pipeline.cpp.

#pragma once

#include <memory>
#include <string>

#include <moveit/global_planner/global_planner_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <rclcpp/rclcpp.hpp>

namespace xarm_hybrid_planning
{
class SceneSyncingPipeline : public moveit::hybrid_planning::GlobalPlannerInterface
{
public:
  SceneSyncingPipeline() = default;
  ~SceneSyncingPipeline() override = default;

  bool initialize(const rclcpp::Node::SharedPtr& node) override;
  bool reset() noexcept override;
  moveit_msgs::msg::MotionPlanResponse
  plan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
      override;

private:
  rclcpp::Node::SharedPtr node_ptr_;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  std::string planning_scene_service_{ "/get_planning_scene" };
  bool sync_before_plan_{ true };
};
}  // namespace xarm_hybrid_planning
