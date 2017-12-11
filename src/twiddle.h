#ifndef TWIDDLE_H_
#define TWIDDLE_H_

#include <string>

class Twiddle {
public:
  Twiddle();

  /*
   * Get PID controller parameter by index
   */
  double GetParam(int index) { return p_[index]; }

  /*
   * Update PID error and control value
   * @param cte cross-track error
   * @param control control value (steering angle)
   */
  void UpdateError(double cte, double control);

  /*
   * Check if the application should terminate in order to
   * run the next twiddle iteration.
   *
   * @return true to terminate, false to continue execution
   */
  bool CheckTerminate();

private:
  /*
   * Twiddle algorithm steps
   */
  enum Step {
    /*
     * Parameter p_[tuneIndex_] should be increased by pd_[tuneIndex_] on the next iteration
     */
    BeforeIncrease,

    /*
     * Parameter p_[tuneIndex_] has been increased by pd_[tuneIndex_]
     */
    AfterDirection1,

    /*
     * The value of pd_[tuneIndex_] has been negated and
     * parameter p_[tuneIndex_] changed by 2 * pd_[tuneIndex_]
     */
    AfterDirection2
  };

  /*
   * Number of tunable parameters
   */
  static constexpr int nparams_ = 3;

  /*
   * Number of frames to skip before starting to track and measure error
   */
  static constexpr int skipframes_ = 1000;

  /*
   * Maximum number of frames to track and measure error
   */
  static constexpr int tuneframes_ = 2000;

  /*
   * Algorithm threshold -- algorithm stops tuning parameters after
   * the sum of absolute values of all parameters goes below the threshold
   */
  static constexpr double threshold_ = 0.1;

  /*
   * Regularization parameter, used to penalize steering angle change
   */
  static constexpr double lambda_ = 1;

  /*
   * Parameters file name
   */
  static const char* fname_;

  /*
   * Parameters backup file name
   */
  static const char* fbakname_;

  /*
   * Number of frames observed so far
   */
  int nframe_ = 0;

  /*
   * Tunable parameters p_ and delta values pd_
   */
  double p_[nparams_];
  double dp_[nparams_];

  /*
   * Index of parameter currently being tuned
   */
  int tuneIndex_ = 0;

  /*
   * Best error observed across all application executions
   */
  double bestError_ = 0;

  /*
   * Running sum of error accumulated during the current execution
   */
  double sumError_ = 0;

  /*
   * Algorithm step
   */
  Step step_;

  /*
   * Current and previous value of control variable
   */
  double control_ = 0;
  double prevControl_ = 0;

  /*
   * Value true indicates that the algorithm has converged, it stops tuning parameters
   * and does not emit a signal to terminate the application
   */
  bool converged_ = false;

  /*
   * Initialize twiddle algorithm values
   */
  void Init();

  /*
   * Load saved state
   */
  void Load(std::ifstream& ifs);

  /*
   * Update step of the algorithm
   */
  void Update();

  /*
   * Save state
   */
  void Save();

  /*
   * Change parameter values when a new best error has been found
   */
  void OnErrorDecrease();

  /*
   * Print state to the output stream
   */
  void PrintState(std::ostream& os);
};

#endif
