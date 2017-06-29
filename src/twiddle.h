#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <array>

#include "PID.h"

enum class TwiddleState {
  PHASE_1,
  PHASE_2,
  PHASE_3
};

class Twiddle {
public:
  double tolerance_;
  double error_;
  double best_error_;
  unsigned int steps_;
  unsigned int processed_steps_;
  std::array<double, 3> parameters_;
  std::array<double, 3> dp_;
  size_t current_param_;
  TwiddleState state_;

  /*
  * Constructor
  */
  Twiddle(PID initial_pid, unsigned int steps, double tolerance);

  /*
  * Destructor
  */
  virtual ~Twiddle();

  /*
  * Registers a new step
  */
  double DoStep(double cte);

private:
  PID pid_;
  bool tolerance_reached_;

  void NextParameter();
  void UpdateControllerParameters();
  void CheckToleranceReached();


};

#endif /* TWIDDLE_H */
