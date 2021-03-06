# Conclusion

## Add the plots to your report and explain them (describe what you see)
The plots can be found in the screenshot folder.  
We have two plots, one for the steering error, which is calculated as the difference between current heading and desired heading direction. The following curves show the output of the steering PID controller (orange) and the steering error (blue), which is used for the control calculations.    
![SteeringPlot](screenshot/Steering.png)   
The second plot below shows the throttle error (blue), i.e., the difference between our current velocity and the desired velocity returned by the trajectory planner. Furthermore, the control output is plotted in green, if the velocity is positive or orange if the velocity is negative and the car brakes.   
![SteeringPlot](screenshot/Throttle.png)  
For both curves we see how the output of the respective controller tries to follow the current error. 

## What is the effect of the PID according to the plots, how each part of the PID affects the control command?
The PID Controller steers the ego vehicle based on the error terms described in the section above. The controller equation  

<img src="https://render.githubusercontent.com/render/math?math=\text{Output} = -K_P \, \text{Err} - K_D \frac{\mathrm{d}}{\mathrm{d}t} \text{Err} -K _I \sum_0^T \text{Err}">

comprises the following three summands:  
### Proportional term
The P term defines the direct influence of the error magnitude on the control output. 
Due to delayed responses of most systems a simple P Controller is bound to overshoot the desired target trajectory, which 
results in an oscillating curve. 
### Differential term
The oscillations can be reduced by adding a differential term who helps the controller to mitigate the overshooting.
### Integral term
The integral term is mainly used to compensate constant offsets. 
The downside of the integration is that the system is slower to respond to sudden changes and also more prone to overshoot. 
I recognized this in the steering Controller, where the steering overshoots whenever the error is present for a longer time period (and therefore the integral term gets larger and larger).


## How would you design a way to automatically tune the PID parameters?
I would try to tune the PIDs independently. 
For the throttle controller I would start with a simulation environment without obstacles and use Twiddle to find optimal parameters as shwon in the lesson. 
Next I would use this controller with constant parameters and try to optimize the parameters of the steering PID. 
For this we need to define a new cost function for Twiddle, because we also have to penalize collisions with cars, walls, .... 
Maybe it would be enough to use the yaw error as cost function + add some large offset if the car collides with a obstacle. 
At last I would try to tune both parameter sets (throttel and steering controller) with Twiddle. Here again collisions are incorporated with a large offset
and the errors in velocity and steering a added with different factors. For the factors I would also try different values, but from intuition I would 
choose a larger factor for steering.  

## PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
Pro: I assume the design is more simple and might work for a larger area of vehicles, as opposed to a Controller which is fine tuned to a certain model
Contra: Through the generalization of PID controller we do not get the best results. I'm sure by incorporating a model we could achieve better results

## (Optional) What would you do to improve the PID controller?
I would try to implement Twiddle as described above and optimize the parameters. 
Furthermore, I would experiment with different error calculation. For example, by implementing the approach of vectorial fields described in the help forum.   

