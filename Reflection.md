# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Description of the effect each of the P, I, D components had in your implementation
Note: Any of the PID controller gains may contribute to making the system instable if they are too high. 


### Description of the effect of the P
Increases wheel angle demand as the vehicle moves away from the center of the road.
The impact of the proportional gain P on the vehicle behaviour is the calculation of a fixed wheel angle demand for a fixed error.
Useful to get the vehicle slowly back towards the center of lane of also to get an immediate turn reaction as a function of the error.

### Description of the effect of the I
Integrates (along time) error from the center of the road to calculate wheel angle demand.
The impact of the integrator gain I on the vehicle behaviour is the calculation of an accumulated wheel demand as a function of a fixed error.
This gain is useful to slowly cancel a static error.

Implementation comment:
I have implemented anti-windup limits that prevents storing large integral values, internal to the filter that may impede the ability of the filter to react quicky to error changes.

### Description of the effect of the D
The derivative gain D helps Computing the wheel angle as function of difference between two successive errors.
This gain is useful to get an immediate turn reaction when the error changes.
This is probably the most important gain for this application in that it controls that the error rate of changes remains close to 0. This means that the gain can be used to ensure that the car remains parallel to the track.

Without this gain, the car is be instable (increasing oscillations in magnitude or diverging error). With a P-controller only, when the car is controlled to the get back to the center of the track, the demanded wheel angle only changes sign when crossing the error is 0 (center of track). This leads to an instable situation because the car is crossing the lane with a high lateral velocity. Instead, we would really want to reach the center of lane with a small lateral velocity. We want the demanded wheel angle to change sign before crossing the center to reduce the rate at which the car approaching it.

For example, see this P only controller: https://youtu.be/2FT3cpWM9pY



Implementation comment:
I implemented a saturation on the D-error to make sure that the calculated gain does not exceed the expected bounds for the wheel angle.
I also implemented a simple Finite Impulse response (FIR) filter to low-pass filter the D-error which tend to diverge very quickly (instable).
A simple protection to divisions by-zero is also implemented. 


### Summing the P-error, I-error and D-error
The total PID effort is calculated by summing the P-error, I-error and D-error.
Because the allowed angles are bounded betweem -1 and 1, I implemented a saturation on the Total PID effort to make sure that the calculated value is within bounds. 


## How the final hyperparameters were chosen

See the video for the final gains here:
https://youtu.be/jp694MZBzIc

### Gain scheduling
Partitioned control space in 4 regions error regions and I allocated to each of the error regions specific gains.
This facilitates controlling the reaction of the car as a function of the pace at which the error changes together with the distance of the car to the center of the track.
When the car is far away from the center of the track, we want a medium-high proportional gain to bring the car slowly back to the center. In addition we want a high derivative gain to prevent the car from drifting away from the center of the track in the event of a strong turn radius.

When the car is close to the center of the track, we want a low proportional gain and a higher integral gain to bring the car very slowly back to the center. In addition we want a low derivative gain to stabilise the vehicle because it is already close the center of track. 

 

### Manual Tuning - 
Tuning inspired by the  Ziegler–Nichols method.
For each of the error regions (see above) and each gains, I manually selected appropriate values:
1. I increased the P-Gain for a fast enough response and a stable system.
2. I increased the I-Gain for a fast enough response and a stable system.
3. I increased the D-Gain for a fast enough response and a stable system.


