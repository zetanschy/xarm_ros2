// Trajectory operator para hybrid planning que engancha cada trayectoria global
// nueva en el waypoint mas cercano al estado REAL del brazo.
//
// Variante de moveit_hybrid_planning/SimpleSampler. Ver la nota de por que en
// nearest_waypoint_sampler.cpp.

#pragma once

#include <cstddef>
#include <string>

#include <moveit/local_planner/trajectory_operator_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>

namespace xarm_hybrid_planning
{
class NearestWaypointSampler : public moveit::hybrid_planning::TrajectoryOperatorInterface
{
public:
  NearestWaypointSampler() = default;
  ~NearestWaypointSampler() override = default;

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
                  const std::string& group_name) override;
  moveit_msgs::action::LocalPlanner::Feedback
  addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory) override;
  moveit_msgs::action::LocalPlanner::Feedback
  getLocalTrajectory(const moveit::core::RobotState& current_state,
                     robot_trajectory::RobotTrajectory& local_trajectory) override;
  double getTrajectoryProgress(const moveit::core::RobotState& current_state) override;
  bool reset() override;

private:
  // Indice del waypoint de la trayectoria de referencia mas cercano a `state`.
  std::size_t nearestWaypoint(const moveit::core::RobotState& state) const;

  std::size_t next_waypoint_index_{ 0 };

  // true cuando llego una trayectoria nueva y todavia no se la anclo al estado
  // actual. El anclaje no se puede hacer en addTrajectorySegment porque esa
  // interfaz no recibe el estado del robot; se hace en el primer
  // getLocalTrajectory posterior, que si lo recibe.
  bool needs_anchor_{ false };

  // Ciclos seguidos sin que el indice avance. Si se pasa del umbral, se vuelve a
  // anclar: es el seguro contra quedarse clavado comandando una pose vieja.
  std::size_t stalled_cycles_{ 0 };

  double waypoint_tolerance_{ 0.2 };   // rad, suma L1 sobre las juntas del grupo
  std::size_t stall_limit_{ 100 };     // ciclos; a 100 Hz es 1 segundo

  moveit_msgs::action::LocalPlanner::Feedback feedback_;
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;
  const moveit::core::JointModelGroup* joint_group_{ nullptr };
  rclcpp::Logger logger_{ rclcpp::get_logger("nearest_waypoint_sampler") };
};
}  // namespace xarm_hybrid_planning
