// Trajectory operator que engancha cada trayectoria global nueva en el waypoint
// mas cercano al estado REAL del brazo.
//
// POR QUE EXISTE
//
// El operador de referencia de MoveIt, SimpleSampler, pone next_waypoint_index_
// = 0 cada vez que adopta una trayectoria nueva. El waypoint 0 es el estado
// inicial que uso el GLOBAL PLANNER, no el estado del brazo en el momento en que
// la solucion llega. Entre una cosa y la otra el brazo se siguio moviendo.
//
// Si esa diferencia supera WAYPOINT_RADIAN_TOLERANCE, el indice no avanza nunca:
// SimpleSampler solo incrementa cuando el estado actual esta cerca del waypoint
// apuntado, y ese waypoint ya quedo atras. El local planner se queda comandando
// para siempre una pose vieja, y si esa pose esta en colision salen los
// "Collision ahead" infinitos.
//
// De ahi que el ciclo reactivo fuera bimodal: o cerraba en 3 replanificaciones,
// o gastaba los 30 reintentos sin avanzar un paso. Medido antes de este operador,
// tres corridas seguidas: OK en 3 intentos, OK en 3 intentos, y una que agoto los
// 30.
//
// COMO LO ARREGLA
//
// Al llegar una trayectoria nueva se marca `needs_anchor_`, y en el primer ciclo
// siguiente -que si recibe el estado del robot- se busca el waypoint mas cercano
// a ese estado y se arranca desde ahi. La diferencia entre el estado del planner
// y el real deja de importar.
//
// Ademas lleva un contador de ciclos sin avanzar. Si el indice se queda quieto
// mas de stall_limit ciclos, se vuelve a anclar. Es el seguro para el caso en que
// el brazo quede lejos de TODOS los waypoints, donde anclar una sola vez no
// alcanzaria y volveriamos al mismo cuelgue.

#include <xarm_hybrid_planning/nearest_waypoint_sampler.h>

#include <algorithm>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

namespace xarm_hybrid_planning
{
bool NearestWaypointSampler::initialize(const rclcpp::Node::SharedPtr& node,
                                        const moveit::core::RobotModelConstPtr& robot_model,
                                        const std::string& group_name)
{
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_name);
  group_ = group_name;
  joint_group_ = robot_model->getJointModelGroup(group_name);
  next_waypoint_index_ = 0;
  needs_anchor_ = false;
  stalled_cycles_ = 0;

  if (node)
  {
    logger_ = node->get_logger();
    // Mismos nombres que usaria un yaml del local planner; si no estan, quedan
    // los valores por defecto, que son los de SimpleSampler.
    waypoint_tolerance_ = node->has_parameter("waypoint_radian_tolerance") ?
                              node->get_parameter("waypoint_radian_tolerance").as_double() :
                              node->declare_parameter<double>("waypoint_radian_tolerance", waypoint_tolerance_);
    const int stall = node->has_parameter("reanchor_stall_cycles") ?
                          node->get_parameter("reanchor_stall_cycles").as_int() :
                          node->declare_parameter<int>("reanchor_stall_cycles", static_cast<int>(stall_limit_));
    stall_limit_ = static_cast<std::size_t>(std::max(1, stall));
  }
  return true;
}

std::size_t NearestWaypointSampler::nearestWaypoint(const moveit::core::RobotState& state) const
{
  std::size_t best = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < reference_trajectory_->getWayPointCount(); ++i)
  {
    const double distance = reference_trajectory_->getWayPoint(i).distance(state, joint_group_);
    if (distance < best_distance)
    {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

moveit_msgs::action::LocalPlanner::Feedback
NearestWaypointSampler::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  reset();

  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory);
  time_parametrization_.computeTimeStamps(*reference_trajectory_);

  // El anclaje se hace en el proximo getLocalTrajectory, que es el que recibe el
  // estado del robot.
  needs_anchor_ = true;
  stalled_cycles_ = 0;

  return feedback_;
}

bool NearestWaypointSampler::reset()
{
  next_waypoint_index_ = 0;
  needs_anchor_ = false;
  stalled_cycles_ = 0;
  reference_trajectory_->clear();
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
NearestWaypointSampler::getLocalTrajectory(const moveit::core::RobotState& current_state,
                                           robot_trajectory::RobotTrajectory& local_trajectory)
{
  if (reference_trajectory_->getWayPointCount() == 0)
  {
    feedback_.feedback = "unhandled_exception";
    return feedback_;
  }

  local_trajectory.clear();

  if (needs_anchor_)
  {
    next_waypoint_index_ = nearestWaypoint(current_state);
    needs_anchor_ = false;
    stalled_cycles_ = 0;
    RCLCPP_INFO(logger_, "trayectoria nueva anclada en el waypoint %zu de %zu (el mas cercano al brazo)",
                next_waypoint_index_, reference_trajectory_->getWayPointCount());
  }

  const moveit::core::RobotState& target = reference_trajectory_->getWayPoint(next_waypoint_index_);

  if (target.distance(current_state, joint_group_) <= waypoint_tolerance_)
  {
    next_waypoint_index_ = std::min(next_waypoint_index_ + 1, reference_trajectory_->getWayPointCount() - 1);
    stalled_cycles_ = 0;
  }
  else if (++stalled_cycles_ > stall_limit_)
  {
    // Clavado apuntando a un waypoint que el brazo no alcanza. Reanclar es
    // preferible a seguir comandando una pose que no se va a alcanzar nunca.
    const std::size_t anchored = nearestWaypoint(current_state);
    RCLCPP_WARN(logger_, "%zu ciclos sin avanzar desde el waypoint %zu; se reancla en el %zu",
                stalled_cycles_, next_waypoint_index_, anchored);
    next_waypoint_index_ = anchored;
    stalled_cycles_ = 0;
  }

  local_trajectory.addSuffixWayPoint(reference_trajectory_->getWayPoint(next_waypoint_index_),
                                     reference_trajectory_->getWayPointDurationFromPrevious(next_waypoint_index_));
  return feedback_;
}

double NearestWaypointSampler::getTrajectoryProgress(const moveit::core::RobotState& /*current_state*/)
{
  if (next_waypoint_index_ >= reference_trajectory_->getWayPointCount() - 1)
  {
    return 1.0;
  }
  return 0.0;
}
}  // namespace xarm_hybrid_planning

PLUGINLIB_EXPORT_CLASS(xarm_hybrid_planning::NearestWaypointSampler,
                       moveit::hybrid_planning::TrajectoryOperatorInterface);
