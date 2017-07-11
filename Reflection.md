# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Describe the effect each of the P, I, D components had in your implementation

### Describe the effect of the P
Increases wheel angle demand as the vehicle moves away from the center of the road.
The impact of the gain P on the vehicle behaviour is the calculation of a fixed wheel angle demand for a fixed error.
Useful to get the vehicle slowly back towards the center of lane of also to get an immediate turn reaction as a function of the error.

### Describe the effect of the I
Integrates (along time) error from the center of the road to calculate wheel angle demand.
The impact of the gain I on the vehicle behaviour is the calculation of an accumulated wheel demand as a function of a fixed error.
Useful to slowly cancel a static error.

Implementation comment:
Filtering and limits...

### Describe the effect of the D
Computes the wheel angle as function of difference between two successive errors.
Useful to get an immediate turn reaction when the error changes.

Implementation comment:
Filtering and limits...


## Describe how the final hyperparameters were chosen

### Manual Tuning - 
Tuning inspired by the  Ziegler–Nichols method...

### Gain scheduling
Partitioned control space in 4 regions...
