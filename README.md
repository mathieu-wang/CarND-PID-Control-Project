# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## PID Gains and Tuning
The PID gains affect the steering angle correction in the following way:
### Proportional (P) gain:
The P gain determines the amount of angle to be corrected based directly on the Cross Track Error (CTE). It is directly multipled with the CTE so that a higher error will result in a bigger correction at the next step. In this context, a P gain that is too low might not correct the angle fast enough and let the vehicle go off-track, especially during sharp turns. If the P gain is too high, the vehicle may overcorrect it's steering angle and oscillate back and forth around the central line, and might even spiral out of control.

### Differential (D) gain:
The D controller adds a component of the correction that depends on the difference between current CTE and last measured CTE. When used in conjunction with the P controller, the D controller smoothes the oscillations that resulted from the P controller. When the D gain is too low, the car will keep oscillating, whereas a D gain that is too high will cancel a lot of the corrections of the P controller, making the vehicle feel unresponsive.

### Integral (I) gain:
The I controller makes corrections based on the amount of error accumulated so far. This ensures that the vehicle does not have any bias, which is especially important when external factors such as drift are present. When the I gain is low, the vehicle might go on a path that has a constant error from the desired path, and might not correct for the bias fast enough. When the I gain is too high, the vehicle will start oscillating again even though P and D gains have already been tuned.

### Tuning steps
The gains were tuned manually based on the observed CTE, the correction steering angle, and the behavior of the vehicle in the simulator. Given the properties of the three gains described above, the following steps are followed to obtain values that make the vehicle drive safely around the track indefinitely.
1. Set all three gains to 0
2. Slowly increase the P gain until it's large enought to fully correct the steering angle on a sharp turn
3. Slowly increase the D gain until it stops most of the oscillation from the P controller without compromising the speed of the correction at sharp turns
4. Slowly increase the I gain until it corrects the bias observed around certain parts of the track without introducing too much oscillations to the vehicle angle corrections.

### Final results
Kp = -0.15
Ki = -0.0015
Kd = -30

### Observations and Learnings
1. One intersting observaition was that the correctiosn would be very abrupt. An analysis of the correction values revealed that it was due to the steering angle correction maxing out at 25 degrees. Bringing down all the gains proportionally helped with smoothing the corrections.

2. Introducing a non-zero I gain actually made the controller oscillate more, but it is necessary in real-world scenarios where drifts and constant error bias need to be corrected.

### Future improvements
1. Use the "Twiddle" algorithm to find the optimal parameters. This could be run with different initial values so local optoma can be avoided.
2. Apply the PID controller with a different set of gains on the speed of the robot, so that it can run faster on straight paths with small error and slower when the error is large and a lot of correction is happening.
