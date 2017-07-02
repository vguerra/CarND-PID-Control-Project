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

Lets set `Kp` to 0.2 and see how our vehicle starts OK, but quickly overshoots, specially as velocity increases.

[![PC](https://img.youtube.com/vi/ht-VbfeFiLU/0.jpg)](https://youtu.be/ht-VbfeFiLU)

So in order to fix this overshooting problem, we consider additional meassurements. This is where Differential Control comes into play.

#### Derivative Control
This term looks at the CTE rate. Meaning, we look at how fast we are moving on the y axis w.r.t reference trayectory. The derivate control term is: `Kp * (CTE - CTE_prev)`. We multiply the difference of CTEs by the *Derivative Gain*. This term helps to tweek the resistance the vehicle feels moving towards the reference trajectory. If the Derivative Gain is too low, then the vehicle will be attracted to the reference trajectory and oscilate. In the contrary, if it is too low, it will take a longer time for the vehicle to correct for offsets. Choosing correctly the Derivative Gain allows the car to approach the desired trajectory quickly in a smooth way.

Lets set `Kd` to 1.5 and see how our vehicle reacts.

[![PDC](https://img.youtube.com/vi/wxl-JDxnwMg/0.jpg)](https://youtu.be/wxl-JDxnwMg)

#### Integral Control
We have seen that setting the Proportional Gain and the Derivative Gain make the vehicle drive OK the simulator track. But we can do better buy integrating a new term that helps the vehicle fight drifting caused by environmental or mechanical deffects. We introduce the integral control term: `Ki * Sum(CTE)`. We multiply the sum of all seen CTEs so far by the *Integral Gain*. The idea behind this term is to penalize the fact that the vehicle could be spending more time on one side of the track than the other w.r.t. the reference trajectory.

Lets set the `Ki` to 0.0009 and see how our vehicle drives this time.

[![PIDC](https://img.youtube.com/vi/ppyhwgzDnKk/0.jpg)](https://youtu.be/ppyhwgzDnKk)

So we have seen that we need to set up 3 parameters to controll all 3 terms that will dictate the steering angle.

#### Choosing parameters

The parameters were tunned manually, starting with all parameters at 0.0 and setting their values in the following order:

* Kd
* Kd
* Ki

[Twiddle was attempted](https://github.com/vguerra/CarND-PID-Control-Project/blob/master/src/twiddle.cpp) but it did not achieved convergence.

### Output Video

This is the output of an entire lap on the simulator.

[![PIDC](https://img.youtube.com/vi/VQa1Z5V68SM/0.jpg)](https://youtu.be/VQa1Z5V68SM)
