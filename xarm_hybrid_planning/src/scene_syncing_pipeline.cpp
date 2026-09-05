// Global planner que PIDE la escena a move_group antes de cada plan.
//
// POR QUE EXISTE
//
// moveit_hybrid_planning/MoveItPlanningPipeline arma su MoveItCpp y con el un
// PlanningSceneMonitor propio. Ese monitor se suscribe a /monitored_planning_scene
// y a /collision_object, pero MoveItCpp NUNCA llama a requestPlanningSceneState():
// la escena del global planner se construye SOLO con los diffs que alcance a
// recibir. Cualquier actualizacion que se pierda -por orden de suscripcion, por
// QoS volatil, por lo que sea- no se recupera nunca, porque no hay un pull que
// vuelva a poner las dos escenas en fase.
//
// Sintoma medido en el demo de la Clase 5: cuando falla, el global planner
// devuelve 27 veces el MISMO plan corto (12-13 waypoints) que va derecho al
// obstaculo, mientras que cuando sale bien devuelve rodeos de 25 a 74 waypoints.
// O sea, planifica como si las placas nuevas no existieran. El local planner, que
// tiene su propio monitor y si las ve, frena; el detector de atasco dispara a los
// 50 ms; se pide replan; y vuelve el mismo plan.
//
// QUE HACE DISTINTO
//
// Antes de planificar llama a requestPlanningSceneState(), que pide la escena
// COMPLETA por el servicio /get_planning_scene de move_group. Es sincrono y no
// depende de haber recibido ningun diff. El resto es identico a
// MoveItPlanningPipeline (no se pudo heredar de el: su moveit_cpp_ es privado).
//
// Se puede apagar con sync_scene_before_plan:=false para comparar en clase.

#include <xarm_hybrid_planning/scene_syncing_pipeline.h>

#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("xarm_hybrid_planning.scene_syncing_pipeline");
const std::string PLANNING_SCENE_MONITOR_NS = "planning_scene_monitor_options.";
const std::string PLANNING_PIPELINES_NS = "planning_pipelines.";
const std::string PLAN_REQUEST_PARAM_NS = "plan_request_params.";
const std::string UNDEFINED = "<undefined>";
}  // namespace

namespace xarm_hybrid_planning
{
bool SceneSyncingPipeline::initialize(const rclcpp::Node::SharedPtr& node)
{
  // Mismos parametros que declara MoveItPlanningPipeline. Hay que declararlos
  // igual porque MoveItCpp los lee del nodo.
  node->declare_parameter<std::vector<std::string>>(PLANNING_PIPELINES_NS + "pipeline_names",
                                                    std::vector<std::string>({ UNDEFINED }));
  node->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "namespace", UNDEFINED);

  node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planner_id", UNDEFINED);
  node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planning_pipeline", UNDEFINED);
  node->declare_parameter<int>(PLAN_REQUEST_PARAM_NS + "planning_attempts", 5);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "planning_time", 1.0);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor", 1.0);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor", 1.0);
  node->declare_parameter<std::string>("ompl.planning_plugin", "ompl_interface/OMPLPlanner");

  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "name", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "robot_description", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "joint_state_topic", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "attached_collision_object_topic", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "publish_planning_scene_topic", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "monitored_planning_scene_topic", UNDEFINED);
  node->declare_parameter<double>(PLANNING_SCENE_MONITOR_NS + "wait_for_initial_state_timeout", 10.0);

  node->declare_parameter<std::string>("moveit_controller_manager", UNDEFINED);

  // Propios
  sync_before_plan_ = node->declare_parameter<bool>("sync_scene_before_plan", true);
  planning_scene_service_ = node->declare_parameter<std::string>("planning_scene_service", planning_scene_service_);

  node_ptr_ = node;

  moveit_cpp::MoveItCpp::Options moveit_cpp_options(node);
  moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node, moveit_cpp_options);

  RCLCPP_INFO(LOGGER, "global planner con sincronizacion de escena %s (servicio %s)",
              sync_before_plan_ ? "ACTIVADA" : "desactivada", planning_scene_service_.c_str());
  return true;
}

moveit_msgs::msg::MotionPlanResponse SceneSyncingPipeline::plan(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
{
  moveit_msgs::msg::MotionPlanResponse response;

  if ((global_goal_handle->get_goal())->motion_sequence.items.empty())
  {
    RCLCPP_WARN(LOGGER, "El global planner recibio una secuencia vacia; hace falta al menos un item.");
    response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return response;
  }
  if ((global_goal_handle->get_goal())->motion_sequence.items.size() > 1)
  {
    RCLCPP_WARN(LOGGER, "Secuencia con mas de un item; se usa solo el primero.");
  }

  // LA diferencia con MoveItPlanningPipeline: traer la escena completa de
  // move_group antes de planificar, en vez de confiar en los diffs recibidos.
  if (sync_before_plan_)
  {
    const auto monitor = moveit_cpp_->getPlanningSceneMonitor();
    if (!monitor || !monitor->requestPlanningSceneState(planning_scene_service_))
    {
      RCLCPP_WARN(LOGGER, "No se pudo sincronizar la escena con %s; se planifica con la que haya.",
                  planning_scene_service_.c_str());
    }
  }

  auto motion_plan_req = (global_goal_handle->get_goal())->motion_sequence.items[0].req;

  moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
  plan_params.planner_id = motion_plan_req.planner_id;
  plan_params.planning_pipeline = motion_plan_req.pipeline_id;
  plan_params.planning_attempts = motion_plan_req.num_planning_attempts;
  plan_params.planning_time = motion_plan_req.allowed_planning_time;
  plan_params.max_velocity_scaling_factor = motion_plan_req.max_velocity_scaling_factor;
  plan_params.max_acceleration_scaling_factor = motion_plan_req.max_acceleration_scaling_factor;

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(motion_plan_req.group_name, moveit_cpp_);
  planning_components->setGoal(motion_plan_req.goal_constraints);

  auto plan_solution = planning_components->plan(plan_params);
  if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    response.error_code = plan_solution.error_code;
    return response;
  }

  response.trajectory_start = plan_solution.start_state;
  response.group_name = motion_plan_req.group_name;
  plan_solution.trajectory->getRobotTrajectoryMsg(response.trajectory);
  response.error_code = plan_solution.error_code;
  return response;
}

bool SceneSyncingPipeline::reset() noexcept
{
  return true;
}
}  // namespace xarm_hybrid_planning

PLUGINLIB_EXPORT_CLASS(xarm_hybrid_planning::SceneSyncingPipeline, moveit::hybrid_planning::GlobalPlannerInterface);
