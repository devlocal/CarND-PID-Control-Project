# CarND-Controls-PID

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

This project implements a PID Controller to estimate the steering angle of a moving vehicle knowing its cross-track error.

---

#### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Project Description

The project contains three groups of files:
1. [main.cpp](src/main.cpp) is the main application file, it contains the code to initialize the program and communicate with the simulator.
1. [PID.cpp](src/PID.cpp) and [PID.h](src/PID.h) contain implementaion of PID controller.
1. [twiddle.cpp](src/twiddle.cpp) and [twiddle.h](src/twiddle.h) contain implementation of Twiddle algorithm for parameter tuning.

#### main.cpp

main.cpp has been changed slightly compared to the initial code. A new variable `twiddle` is added (line 37) which is an instance of `Twiddle` class that performs parameter tuning for the PID controller.

main.cpp updates PID controller with the current value of CTE by calling `pid.UpdateError(cte)` (line 60) which is followed by getting the value of steering angle (line 61).

After these steps, Twiddle algorithm is called to update CTE value and steering angle value (line 64). The next step is to check if Twiddle is finished the current iteration of tuning and the application should be re-started for the next iteration (lines 65-69).

Computed value of steering angle is kept within [-1, 1] range (lines 71-82).

#### PID Controller Implementation

PID Controller is initialized with values of `Kp`, `Ki` and `Kd` (lines 13-16 of [PID.cpp](src/PID.cpp)).

Function `GetControlValue` computes the new value of steering angle (line 19).

Function `UpdateError` accumulates all error measurements and keeps track of the previous error value (lines 23-32). The first incoming error measurement initializes both the previous and the current cross-track error with the same value.

#### Twiddle algorithm

Twiddle algorithm is implemented very close to the method described in the lesson. It starts with initial values of parameters `P`, `I` and `D` (stored in `p_` array, [twiddle.h](src/twiddle.h), line 96) and delta values for each parameter (stored in `dp_` array, line 97). It works sequentially by trying to decrease or increase each PID parameter and iterating over three parameters in a loop until convergence. The algorithm stops when the sum of absolute values of `dp_` decreases below a threshold (`threshold_`, line 71).

Two major differences to the algorithm described in the lecture are:
1. The algorithm cannot tune parameters in a single program execution. In order to function, it saves parameters in a JSON file and requires the application to be restarted. Upon restart, it loads updated parameters from the state file and measures the error. Simulator needs to be restarted every time as well, so the car drives over the same path on every execution.
2. A new regularization term `lambda_` has been added ([twiddle.h](src/twiddle.h), line 76). It is used to penalize a set of parameters that causes frequent changes in steering angle.

The algorithm initializes in the constructor ([twiddle.cpp](src/twiddle.cpp), lines 16-24), if a state file is available then `Load` is called to restore the state, otherwise state is initialized by calling `Init`. Function `Init` sets algorithm values to a known good set of initial values found empirically (lines 26-36).

Functions `Load` (lines 38-59) and `Save` (lines 61-86) leverage [json.hpp](src/json.hpp) to persist algorithm state in `twiddle.json` file. The final set of parameters learned by the algorithm is available in [build/twiddle.json](build/twiddle.json):

```javascript
{
  "bestError": 51.315526193673,
  "p0": 0.1900413983518,
  "p1": 0.000844726355188655,
  "p2": 4.92048761910001,
  "pd0": 0.0205054376539538,
  "pd1": 2.3204550188655e-05,
  "pd2": 0.0565167755737025,
  "step": 1,
  "tuneIndex": 1
}
```

The file contains the following values of PID controller parameters which allow the car to drive within the drivable area of the track:

| P    | I       | D    |
|:----:|:-------:|:----:|
| 0.19 | 0.00084 | 4.92 |

Function `UpdateError` (lines 98-114) increases the running sum of the driving error `sumError_` which is computed as

```c
sumError_ += cte * cte + pow(control_ - prevControl_, 2) * lambda_;
```

where `sumError_` - accumulated error, `cte` - cross-track error, `control_` - steering angle, `prevControl_` - previous value of steering angle, `lambda_` - regularization parameter.

The function `CheckTerminate` (lines 116-143) is called by main.cpp, it uses its return value to indicate when the program should terminate. The function enters its main part either when enough measurements have been seen, or if accumulated error exceeds the best known error. It computes the sum of absolute values of parameter deltas (lines 120-123), if the computed value is below `threshold_` the function sets `converged_` to `true`, and application is kept running the without further parameter tuning. Otherwise it calls `Save` and `Update` and returns `false` to stop the application.

Function `Update` is the core part of the Twiddle algorithm. It implements the algorithm as describes in the lesson with the difference that the state is carried over from one execution of the program to another.

### Effect of P, I, D components

Each component of the PID controller parameters play its part in computing the right steering angle for a smooth and accurate driving.

Setting `P` component to `0.19` and other two components to zero results in the car oscillating from left to right with increasing amplitude which eventually brings the car outside of the track. The car behavior with the `P` component only is captured in [media/only_p.mov](media/only_p.mov).

Setting `D` component to `4.92` and two other components to zero results in an inaccurate off-center driving as well. The car is less reactive than with the `P` component alone. As the car diverts from the center of the track, the differential part of the PID controller increases the steering angle, however it can't bring the car back to the center of the track. The car eventually goes off-track. The car behavior with the `D` component only is captured in [media/only_d.mov](media/only_d.mov).

Setting `I` component to `0.00084` and two other components to zero results in unacceptable driving. As the car diverges from the center, PID controller brings the car back, however it overshoots, the car goes off-track and is never able to return back. The car behavior with the `I` component only is captured in [media/only_i.mov](media/only_i.mov).

Setting `I` component to zero while keeping two other components `P` and `D` to their best known values does not cause much of a noticable impact to the driving compared to using all three PID components. This can be explained by relatively low value of the `I` component, which may indicate a high accuracy of the simulated car steering mechanics.

### Driving Video

A video of one lap of driving with PID components set to `P=0.19`, `I=0.00084`, `D=4.92` is available in [media/one_lap.mov](media/one_lap.mov).