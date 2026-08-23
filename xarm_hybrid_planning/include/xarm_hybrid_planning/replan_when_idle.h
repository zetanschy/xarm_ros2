// Planner logic plugin para el hybrid planning del curso.
//
// Variante de moveit_hybrid_planning/ReplanInvalidatedTrajectory que arregla dos
// cosas que impiden que el ciclo reactivo cierre:
//
//   1. No le pide un plan nuevo al global planner si ya hay uno en vuelo. El
//      plugin original pide uno por cada evento del local planner, y como
//      LOCAL_PLANNER_STUCK se re-arma solo (forward_trajectory.cpp resetea
//      num_iterations_stuck_ al emitirlo), llegan ~21 por segundo. Cada pedido
//      nuevo preempta el anterior, asi que ninguno llega a terminar y nunca
//      aparece una trayectoria nueva.
//
//   2. No trata como fatal un 'Global planning action aborted'. Con el plugin
//      original, en cuanto un pedido preempta a otro el aborto resultante cae en
//      el 'else' de react() y mata todo el hybrid planning.
//
// Ademas corta con un error claro despues de max_replan_attempts, para que la
// demo termine siempre en vez de reintentar para siempre.

#pragma once

#include <chrono>

#include <moveit/hybrid_planning_manager/planner_logic_interface.h>
#include <moveit/hybrid_planning_manager/hybrid_planning_manager.h>

namespace xarm_hybrid_planning
{
class ReplanWhenIdle : public moveit::hybrid_planning::PlannerLogicInterface
{
public:
  ReplanWhenIdle() = default;
  ~ReplanWhenIdle() override = default;

  bool initialize(const std::shared_ptr<moveit::hybrid_planning::HybridPlanningManager>&
                      hybrid_planning_manager) override;
  moveit::hybrid_planning::ReactionResult
  react(const moveit::hybrid_planning::HybridPlanningEvent& event) override;
  moveit::hybrid_planning::ReactionResult react(const std::string& event) override;

private:
  // Pide un plan global si no hay ninguno en vuelo. Devuelve false si el envio
  // fallo (no si se ignoro por estar ocupado).
  bool requestGlobalPlanIfIdle();

  bool local_planner_started_ = false;
  bool global_planning_in_flight_ = false;
  int replan_attempts_ = 0;
  std::chrono::steady_clock::time_point last_request_time_{};

  // Intervalo minimo entre pedidos al global planner.
  //
  // La exclusion mutua sola no alcanza: el global planner resuelve esto en ~40 ms
  // y LOCAL_PLANNER_STUCK se re-arma cada pocas decenas de ms mientras el brazo
  // este quieto, asi que sin limite de tasa se gastan todos los reintentos en un
  // segundo aunque el brazo este avanzando bien. Un lazo reactivo necesita
  // control de tasa, no solo exclusion mutua.
  static constexpr std::chrono::milliseconds MIN_REPLAN_INTERVAL{ 500 };

  // Tope de reintentos, para que la demo termine si de verdad no hay salida.
  static constexpr int MAX_REPLAN_ATTEMPTS = 30;
};
}  // namespace xarm_hybrid_planning
