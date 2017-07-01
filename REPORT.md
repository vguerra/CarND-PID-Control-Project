# PID Controller

* [The Goal](#the-goal)
* [Rubric](#rubric)
* [Code](#code)
* [PID Controller](#pid-controller)
* [Output Video](#output-video)

---

## The Goal

For this project the goal is to implement a PID Controller to maneuver the vehicle around the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

---

## [Rubric](https://review.udacity.com/#!/rubrics/824/view)
Here I will consider the rubric points individually and describe how I addressed each point in my implementation. 

---

### Code
The project is organized as follows:
* [`src`](https://github.com/vguerra/CarND-PID-Control-Project/tree/master/src): Folder containing the source files.
    - [`main.cpp`](https://github.com/vguerra/CarND-PID-Control-Project/blob/master/src/main.cpp): Implements communication with simulator and pass the needed parameters received from the Simulator into the PID controller implementation.
    - [`PID.cpp`](https://github.com/vguerra/CarND-PID-Control-Project/blob/master/src/PID.cpp): Implements PID Controller.

### PID Controller

The PID controller will yield the value for the steering angle we will use in our vehicle. It takes into account the CTE, which is the Cross Track Error. It tell's us how far away our vehicle is from the desired trajectory.

The PID Controller is composed of 3 parts:

#### Proportional Control
The idea is to have the vehicle to steer harder the farther away we are from the desired trajectory. Meaning that our steering angle is proportional to the CTE. The proportional control term is: `Kp * CTE` where Kp is the **Proportional Gain**. The perfomance of our vehicle gets better as the gain increases. But having alone Proportional Control is not enough in some situations, for instance, when we start too far away from the desired trajectory. Another disavantage of using only Proportional Control is that our vehicle will constantly overshoot.

As an example we set the `Kp` to 0.2 and see how our vehicle starts OK, but quickly starts to overshoot, specially as velocity increases.

[![Project Video](https://img.youtube.com/vi/ht-VbfeFiLU/0.jpg)](https://youtu.be/ht-VbfeFiLU)
### Output Video