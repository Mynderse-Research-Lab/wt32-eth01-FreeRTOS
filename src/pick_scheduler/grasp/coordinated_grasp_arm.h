/**
 * @file coordinated_grasp_arm.h
 * @brief Bench arm that fires a coordinated-grasp plan.
 *
 * Included from coordinated_grasp.h after the model types.
 */
#ifndef COORDINATED_GRASP_ARM_H
#define COORDINATED_GRASP_ARM_H

namespace CoordinatedGrasp {

// Bench arm. Production leaves this disarmed. pk_dropin is not consulted.
struct Arm {
  uint32_t generation = 0;
  bool armed = false;
  bool z_pending = false;
  bool close_pending = false;
  bool z_fired = false;
  bool close_fired = false;
  int64_t z_at_us = 0;
  int64_t close_at_us = 0;
  int64_t retract_at_us = 0;
  Plan plan{};
};

inline void disarm(Arm& arm) {
  arm.armed = false;
  arm.z_pending = false;
  arm.close_pending = false;
}

inline void armPair(Arm& arm, const Plan& plan, int64_t t_a_us, uint32_t generation) {
  disarm(arm);
  arm.generation = generation;
  arm.plan = plan;
  if (plan.status != Status::AnalyticClear && plan.status != Status::Tighten) {
    return;
  }
  arm.z_at_us = t_a_us + static_cast<int64_t>(std::llround(plan.t_z_s * 1.0e6));
  arm.close_at_us = t_a_us + static_cast<int64_t>(std::llround(plan.t_close_s * 1.0e6));
  arm.retract_at_us = t_a_us + static_cast<int64_t>(std::llround(plan.t_grasp_s * 1.0e6));
  arm.z_pending = true;
  arm.close_pending = true;
  arm.armed = true;
}

inline bool fireDue(Arm& arm, uint32_t generation, int64_t now_us, bool want_z) {
  if (!arm.armed || generation != arm.generation) {
    return false;
  }
  if (want_z) {
    if (!arm.z_pending || now_us + 1000 < arm.z_at_us) {
      return false;
    }
    arm.z_pending = false;
    arm.z_fired = true;
    return true;
  }
  if (!arm.close_pending || now_us + 1000 < arm.close_at_us) {
    return false;
  }
  arm.close_pending = false;
  arm.close_fired = true;
  return true;
}

}  // namespace CoordinatedGrasp

#endif  // COORDINATED_GRASP_ARM_H
