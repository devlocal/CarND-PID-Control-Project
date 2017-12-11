#include <fstream>
#include <strstream>
#include "twiddle.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

const char* Twiddle::fname_ = "twiddle.json";
const char* Twiddle::fbakname_ = "twiddle.bak.json";

/**
 * Twiddle algorithm implementation
 */

Twiddle::Twiddle() {
  // Load state from file if file exists, initialize otherwise
  std::ifstream ifs(fname_);
  if (ifs.good()) {
    Load(ifs);
  } else {
    Init();
  }
}

void Twiddle::Init() {
  step_ = BeforeIncrease;

  // Initial values found empirically
  p_[0] = 1;
  p_[1] = 0.0001;
  p_[2] = 5;
  dp_[0] = 0.1;
  dp_[1] = 0.00005;
  dp_[2] = 0.5;
}

void Twiddle::Load(std::ifstream& ifs) {
  json j;
  ifs >> j;

  bestError_ = j["bestError"];
  tuneIndex_ = j["tuneIndex"];
  step_ = j["step"];

  for (int i = 0; i < nparams_; ++i) {
    std::ostringstream pName;
    std::ostringstream pdName;
    pName << "p" << i;
    pdName << "pd" << i;

    p_[i] = j[pName.str()];
    dp_[i] = j[pdName.str()];
  }

  std::cout << "Loaded Twiddle state:";
  PrintState(std::cout);
  std::cout << std::endl;
}

void Twiddle::Save() {
  json j;

  j["bestError"] = bestError_;
  j["tuneIndex"] = tuneIndex_;
  j["step"] = step_;

  for (int i = 0; i < nparams_; ++i) {
    std::ostringstream pName;
    std::ostringstream pdName;
    pName << "p" << i;
    pdName << "pd" << i;

    j[pName.str()] = p_[i];
    j[pdName.str()] = dp_[i];
  }

  rename(fname_, fbakname_);

  std::ofstream ofs(fname_);
  ofs << j.dump(2);

  std::cout << "Saved Twiddle state:";
  PrintState(std::cout);
  std::cout << std::endl;
}

void Twiddle::PrintState(std::ostream &os) {
  for (int i = 0; i < nparams_; ++i) {
    std::cout << " p" << i << "=" << p_[i];
  }
  for (int i = 0; i < nparams_; ++i) {
    std::cout << " dp" << i << "=" << dp_[i];
  }
  std::cout << " index=" << tuneIndex_ << " step=" << step_ << " bestError=" << bestError_;
}

void Twiddle::UpdateError(double cte, double control) {
  // Haven't seen enough frames yet?
  if (nframe_ <= tuneframes_) {
    // Initial frames skipped?
    if (nframe_ >= skipframes_) {
      // Accumulate squared value of cross-track error and squared value of
      // change in control variable to favor slight off-track driving over
      // frequent steering angle changes.
      sumError_ += cte * cte + pow(control_ - prevControl_, 2) * lambda_;
    }

    ++nframe_;
  }

  prevControl_ = control_;
  control_ = control;
}

bool Twiddle::CheckTerminate() {
  bool stopEarly = step_ != BeforeIncrease && sumError_ >= bestError_;
  if (!converged_ && (nframe_ >= tuneframes_ || stopEarly)) {
    // Check if the algorithm has converged
    double s = 0;
    for (double i : dp_) {
      s += abs(i);
    }

    if (s < threshold_) {
      std::cout << "Twiddle converged to";
      for (int i = 0; i < nparams_; ++i) {
        std::cout << " p" << i << "=" << p_[i];
      }
      std::cout << std::endl;
      converged_ = true;
    } else {
      if (step_ == BeforeIncrease) {
        bestError_ = sumError_;
      }
      Update();
      Save();
      return true;
    }
  }

  return false;
}

void Twiddle::Update() {
  do {
    switch (step_) {
      /*
       * BeforeIncrease is called either on the very first program execution
       * to measure the current value of error or after trying to change
       * a parameter in both directions without being able to reduce the error
       */
      case BeforeIncrease:
        p_[tuneIndex_] += dp_[tuneIndex_];
        step_ = AfterDirection1;
        break;

      /*
       * Called after parameter value has been changed in one direction
       * and a new error has been measured
       */
      case AfterDirection1:
        if (sumError_ < bestError_) {
          OnErrorDecrease();
        } else {
          dp_[tuneIndex_] = -dp_[tuneIndex_];
          p_[tuneIndex_] += 2 * dp_[tuneIndex_];
          step_ = AfterDirection2;
        }
        break;

      /*
       * Called after parameter value has been changed in the opposite direction
       * and a new error has been measured
       */
      case AfterDirection2:
        if (sumError_ < bestError_) {
          OnErrorDecrease();
        } else {
          p_[tuneIndex_] -= dp_[tuneIndex_];
          dp_[tuneIndex_] *= 0.9;
          tuneIndex_ = (tuneIndex_ + 1) % nparams_;
          step_ = BeforeIncrease;
        }
        break;
    }
  } while (step_ == BeforeIncrease);
}

void Twiddle::OnErrorDecrease() {
  bestError_ = sumError_;
  dp_[tuneIndex_] *= 1.1;
  tuneIndex_ = (tuneIndex_ + 1) % nparams_;
  step_ = BeforeIncrease;
}