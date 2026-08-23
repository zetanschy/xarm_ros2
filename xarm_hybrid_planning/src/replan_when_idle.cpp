#include <xarm_hybrid_planning/replan_when_idle.h>

#include <moveit/local_planner/feedback_types.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("xarm_hybrid_planning.replan_when_idle");
}

namespace xarm_hybrid_planning
{
using moveit::hybrid_planning::HybridPlanningEvent;
using moveit::hybrid_planning::LocalFeedbackEnum;
using moveit::hybrid_planning::ReactionResult;
using moveit::hybrid_planning::toString;
using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

bool ReplanWhenIdle::initialize(
    const std::shared_ptr<moveit::hybrid_planning::HybridPlanningManager>& hybrid_planning_manager)
{
  hybrid_planning_manager_ = hybrid_planning_manager;
  return true;
}

bool ReplanWhenIdle::requestGlobalPlanIfIdle()
{
  if (global_planning_in_flight_)
  {
    // Exclusion mutua: sin esto, cada evento del local planner dispara un pedido
    // que preempta al anterior y ninguno termina nunca.
    RCLCPP_DEBUG(LOGGER, "Ya hay una planificacion global en vuelo; se ignora el evento.");
    return true;
  }

  // Limite de tasa: LOCAL_PLANNER_STUCK se re-arma cada pocas decenas de ms
  // mientras el brazo este quieto. Sin esto se gastan todos los reintentos en un
  // segundo, incluso cuando el brazo esta avanzando por un desvio nuevo.
  const auto now = std::chrono::steady_clock::now();
  if (last_request_time_.time_since_epoch().count() != 0 && now - last_request_time_ < MIN_REPLAN_INTERVAL)
  {
    RCLCPP_DEBUG(LOGGER, "Pedido demasiado seguido; se ignora el evento.");
    return true;
  }
  last_request_time_ = now;

  ++replan_attempts_;
  if (replan_attempts_ > MAX_REPLAN_ATTEMPTS)
  {
    RCLCPP_WARN(LOGGER, "Se agotaron los %d reintentos de replanificacion.", MAX_REPLAN_ATTEMPTS);
    return false;
  }

  RCLCPP_INFO(LOGGER, "Pidiendo plan global (intento %d de %d).", replan_attempts_, MAX_REPLAN_ATTEMPTS);
  if (!hybrid_planning_manager_->sendGlobalPlannerAction())
  {
    return false;
  }
  global_planning_in_flight_ = true;
  return true;
}

ReactionResult ReplanWhenIdle::react(const HybridPlanningEvent& event)
{
  switch (event)
  {
    case HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED:
      local_planner_started_ = false;
      global_planning_in_flight_ = false;
      replan_attempts_ = 0;
      last_request_time_ = {};
      if (!requestGlobalPlanIfIdle())
      {
        hybrid_planning_manager_->sendHybridPlanningResponse(false);
      }
      return ReactionResult(event, "", ErrorCodes::SUCCESS);

    case HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE:
      // Se espera a que la accion termine, igual que SinglePlanExecution.
      return ReactionResult(event, "", ErrorCodes::SUCCESS);

    case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_SUCCESSFUL:
      global_planning_in_flight_ = false;
      if (!local_planner_started_)
      {
        if (!hybrid_planning_manager_->sendLocalPlannerAction())
        {
          hybrid_planning_manager_->sendHybridPlanningResponse(false);
        }
        local_planner_started_ = true;
      }
      // Si el local planner ya estaba corriendo, no hay que rearrancarlo: el
      // propio local planner adopta la trayectoria nueva por el topico
      // global_trajectory (SimpleSampler::addTrajectorySegment la reemplaza).
      return ReactionResult(event, "", ErrorCodes::SUCCESS);

    case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_ABORTED:
    case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_CANCELED:
      global_planning_in_flight_ = false;
      if (!local_planner_started_)
      {
        // Todavia no hay nada ejecutandose: no hubo primera solucion, se aborta.
        return ReactionResult(event, "El global planner no encontro una solucion inicial",
                              ErrorCodes::PLANNING_FAILED);
      }
      // Ya estamos ejecutando. Un aborto aca casi siempre es un pedido nuestro
      // preemptando al anterior, y NO es motivo para matar el hybrid planning:
      // si la trayectoria sigue invalida, el local planner volvera a avisar.
      RCLCPP_INFO(LOGGER, "Planificacion global abortada durante la ejecucion; se sigue.");
      return ReactionResult(event, "", ErrorCodes::SUCCESS);

    case HybridPlanningEvent::LOCAL_PLANNING_ACTION_SUCCESSFUL:
      hybrid_planning_manager_->sendHybridPlanningResponse(true);
      return ReactionResult(event, "", ErrorCodes::SUCCESS);

    case HybridPlanningEvent::LOCAL_PLANNING_ACTION_ABORTED:
      return ReactionResult(event, "El local planner aborto", ErrorCodes::PLANNING_FAILED);

    case HybridPlanningEvent::LOCAL_PLANNING_ACTION_CANCELED:
      return ReactionResult(event, "", ErrorCodes::SUCCESS);

    default:
      // A diferencia de los plugins de MoveIt, un evento desconocido NO es fatal.
      RCLCPP_DEBUG(LOGGER, "Evento no manejado; se ignora sin abortar.");
      return ReactionResult(event, "", ErrorCodes::SUCCESS);
  }
}

ReactionResult ReplanWhenIdle::react(const std::string& event)
{
  if (event == toString(LocalFeedbackEnum::COLLISION_AHEAD) ||
      event == toString(LocalFeedbackEnum::LOCAL_PLANNER_STUCK))
  {
    if (!requestGlobalPlanIfIdle())
    {
      return ReactionResult(event, "Se agotaron los reintentos de replanificacion",
                            ErrorCodes::PLANNING_FAILED);
    }
    return ReactionResult(event, "", ErrorCodes::SUCCESS);
  }

  RCLCPP_DEBUG(LOGGER, "Evento de texto no manejado: '%s'; se ignora.", event.c_str());
  return ReactionResult(event, "", ErrorCodes::SUCCESS);
}
}  // namespace xarm_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(xarm_hybrid_planning::ReplanWhenIdle,
                       moveit::hybrid_planning::PlannerLogicInterface)
