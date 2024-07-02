## PID Controller Method

The first evolution of the PID controller was developed in 1911 by Elmer Sperry. However, it
wasn't until 1933 that the Taylor Instrumental Company (TIC) introduced the first pneumatic
controller with a fully tunable proportional controller. A few years later, control engineers went
eliminate the steady state error found in proportional controllers by resetting the point to some
artificial value as long as the error wasn’t zero. This resetting “integrated” the error and became
known as the proportional-Integral controller. Then, in 1940, TIC developed the first PID
pneumatic controller with a derivative action, which reduced overshooting issues. However, it
wasn’t until 1942, when Ziegler and Nichols tuning rules were introduced that engineers were
able to find and set the appropriate parameters of PID controllers.
PID controller is universally accepted and most commonly used controller in industrial
application because PID controller is simple, provide good stability and rapid response. PID
stands for proportional, integral, derivative. In each application, coefficient of these three
actions are varied to get optimal response and control. Controller input is error signal. And
output is given to the plant/process. Output signal of controller is generated, in such a way that,
output of plant is try to achieve desired value. PID controller is a Close loop system which has
feedback control system and it compares the Process variable (feedback variable) with set Point
and generates an error signal and according to that it adjusts the output of system. This process
continues until this error gets to Zero or process variable value becomes equal to set point in :


![Uploading Screenshot from 2024-07-02 15-24-14.png…]()



At present, there are various kinds of PID controllers are available in the market. These
controllers are used for industrial control requirements like pressure, temperature, level, and
flow. Before the working of the PID controller takes place, it must be tuned to suit with
dynamics of the process to be controlled. Designers give the default values for P, I, and D
terms, and these values couldn’t give the desired performance and sometimes leads to
instability and slow control performances. Different types of tuning methods are developed to
tune the PID controllers and require much attention from the operator to select the best values
of proportional, integral, and derivative gains.
