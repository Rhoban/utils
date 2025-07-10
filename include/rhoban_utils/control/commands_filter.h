#pragma once

#ifndef LOW_PASS_COMMANDS_FILTER_H
#define LOW_PASS_COMMANDS_FILTER_H

#include <Eigen/Dense>

class LowPassCommandsFilter {
public:
  LowPassCommandsFilter(double control_freq, double cutoff_frequency = 30.0);

  void push(const Eigen::Vector3d &action);

  Eigen::Vector3d get_filtered_command();
  double cutoff_frequency;
  double compute_alpha();

private:

  double control_freq;
  double alpha;

  Eigen::Vector3d last_action;
  Eigen::Vector3d current_action;
};

#endif // LOW_PASS_COMMANDS_FILTER_H
