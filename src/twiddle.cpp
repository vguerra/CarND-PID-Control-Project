#include <numeric>
#include <iostream>
#include <cmath>


#include "twiddle.h"

Twiddle::Twiddle(PID initial_pid,
                 unsigned int steps,
                 double tolerance) : tolerance_(tolerance), error_(0.0), steps_(steps), processed_steps_(0), tolerance_reached_(false) {

  pid_ = initial_pid;

  // move this to its own method? might be reused later.
  parameters_ = { {pid_.Kp, pid_.Ki, pid_.Kd} };
  dp_ = { {1.0, 1.0, 1.0} };
  best_error_ = std::numeric_limits<double>::max();
  current_param_ = 0;
  state_ = TwiddleState::PHASE_1;
}

Twiddle::~Twiddle() {}

double Twiddle::DoStep(double cte) {

  if (tolerance_reached_) {
    pid_.UpdateError(cte);
    return pid_.TotalError();
  }

  if (processed_steps_ == 0) {
    error_ = 0.0;

    switch (state_) {
      case TwiddleState::PHASE_1:
        parameters_[current_param_] += dp_[current_param_];
        break;
      case TwiddleState::PHASE_2:
        parameters_[current_param_] -= 2*dp_[current_param_];
        break;
      case TwiddleState::PHASE_3:
        break;
    }

    UpdateControllerParameters();

  } else if (processed_steps_ == steps_ || std::fabs(cte) > 4.0) {
    error_ /= processed_steps_;

    processed_steps_ = 0;

    std::cout << "best error : " << best_error_ << ", error : " << error_ << '\n';

    switch (state_) {

      case TwiddleState::PHASE_1:

        if (error_ < best_error_) {
          best_error_ = error_;
          dp_[current_param_] *= 1.1;
          NextParameter();
        } else {
          state_ = TwiddleState::PHASE_2;
        }
        break;

      case TwiddleState::PHASE_2:

        state_ = TwiddleState::PHASE_1;

        if (error_ < best_error_) {
          best_error_ = error_;
          dp_[current_param_] *= 1.1;
        } else {
          dp_[current_param_] *= 0.9;
          parameters_[current_param_] += dp_[current_param_];
        }
        NextParameter();
        break;
      case TwiddleState::PHASE_3:
        break;
    }

    CheckToleranceReached();
    return 1000.0;
  }

  pid_.UpdateError(cte);
  error_ += cte * cte;

  processed_steps_ += 1;

  return pid_.TotalError();
}

void Twiddle::NextParameter() {
  current_param_ += 1;
  current_param_ %= 3;
}

void Twiddle::UpdateControllerParameters() {
  pid_.Init(parameters_[0], parameters_[1], parameters_[2]);
  std::cout << pid_.Kp << ", " << pid_.Ki << ", " << pid_.Kd << '\n';
}

void Twiddle::CheckToleranceReached() {
  tolerance_reached_ = std::accumulate(begin(dp_), end(dp_), 0.0) < tolerance_;
}
