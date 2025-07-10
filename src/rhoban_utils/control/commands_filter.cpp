#include "rhoban_utils/control/commands_filter.h"

LowPassCommandsFilter::LowPassCommandsFilter(double control_freq,
                                             double cutoff_frequency)
    : control_freq(control_freq), cutoff_frequency(cutoff_frequency),
      last_action(Eigen::Vector3d::Zero()),
      current_action(Eigen::Vector3d::Zero()) {
}

double LowPassCommandsFilter::compute_alpha(){
  return (1.0 / cutoff_frequency) /
         ((1.0 / control_freq) + (1.0 / cutoff_frequency));
}

void LowPassCommandsFilter::push(const Eigen::Vector3d &action) {
  current_action = action;
}

Eigen::Vector3d LowPassCommandsFilter::get_filtered_command() {
  alpha = compute_alpha();
  last_action = alpha * last_action + (1.0 - alpha) * current_action;
  return last_action;
}
