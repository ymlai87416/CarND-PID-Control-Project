
[//]: # (Image References)

[image1]: ./output/equation.PNG "PID equation"
[image2]: ./output/Kd.PNG "Kd"
[image3]: ./output/Ki.PNG "Ki"
[image4]: ./output/Kp.PNG "Kp"
[image5]: ./output/PID_graph.png "PID graphical representation"
[image6]: ./output/controllers.png "PID controllers"

# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.


### PID implementation

In this project, I have implemented a robot having 2 PID controllers, one for controlling the speed and one for controlling the steering angle.

The PID controller for steering angle is used to keep the car on the track at a constant speed (50 mph), but using this PID controller alone, 
the car may not make a turn safely, so there is a need for another PID controller to control the speed of the car.

Normally the car tries to achieve a speed of 60mph, if the error term of the steering angle is too high, the speed is reduced to 
minimum 33mph . The robot sets the target speed and the PID controller controlling the speed accelerates or decelerates the car.

### Effect of the P, I, D components have in the implementation

PID graphical representation

![alt text][image5]

The control function is of this form

![alt text][image1]

Where ![alt text][image4], ![alt text][image3] and ![alt text][image2] are all non-negative, denote the coefficient of proportional, integral, derivative terms.

P: proportional to the current value of error. If there is an error, the control output will be proportionate to the error, taking into the account the gain factor Kp

I: account for the past error value and integrates them over time. This term seeks to eliminate the residual error by adding a control effect due to the historic cumulative value of the
error. When the error is first eliminated, the integral term is not zero and hence oscillate a bit before settling down.

D: this term seeks to reduce the effect of the error by exerting a control influence generated by the rate of error change. The more rapid the change, the greater the controlling or dampening effect.

Let see the effect of P controller, PD controller, and PID controller on a step change (from 1 to 0).

![alt text][image6]


### To choose the final hyperparameters

In this project, I use the twiddle algorithm to find a set of hyperparameters. The implementation is written in ```twiddle.cpp```.

The implementation makes use of the simulator. It set the PID controller to an initial value, and run the twiddle algorithm, each 
combination of Kp, Ki and Kd are used to control the car for say 20s, and record the error. There are cases where the car run off the road,
in these cases, I restart the program with the best Kp, Ki ,and Kd I have found in the previous run.

The final hyperparameters I have found is as followed:

|         | ![alt text][image4]   | ![alt text][image3] | ![alt text][image2] |
|:-----:| :-----:| :-----:| :-----:|
| Steering | 0.13715 | 0.0002398 | 2.95 |
| Speed | 0.2 | 0 | 0.02 |

### Simulation result

Here is the [result](https://youtu.be/tF9NMcee8iA) 
