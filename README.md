## 1. PID Controller Method

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


   ![Screenshot from 2024-07-02 15-24-14](https://github.com/TepmarotdanielZ/PID/assets/139426571/6b06277a-bc69-4557-bb4f-529a4dd78c42)



At present, there are various kinds of PID controllers are available in the market. These
controllers are used for industrial control requirements like pressure, temperature, level, and
flow. Before the working of the PID controller takes place, it must be tuned to suit with
dynamics of the process to be controlled. Designers give the default values for P, I, and D
terms, and these values couldn’t give the desired performance and sometimes leads to
instability and slow control performances. Different types of tuning methods are developed to
tune the PID controllers and require much attention from the operator to select the best values
of proportional, integral, and derivative gains.

## 2. DC Encoder Motor

A DC motor is an electric motor that runs on direct current power. In an electric motor, the
operation is dependent upon simple electromagnetism. A current-carrying conductor generates
a magnetic field, when this is then placed in an external magnetic field, it will encounter a force
proportional to the current in the conductor and to the strength of the external magnetic field.
It is a device that converts electrical energy to mechanical energy. It works on the fact that a
current-carrying conductor placed in a magnetic field experiences a force that causes it to rotate
with respect to its original position. Practical DC Motor consists of field windings to provide
the magnetic flux and armature which acts as the conductor.
An encoder is an electromechanical device that provides an electrical signal Figure 2-8 that is
used for speed and/or position control. Encoders turn mechanical motion into an electrical
signal that is used by the control system to monitor specific parameters of the application and
make adjustments if necessary, to maintain the machine operating as desired.


![Screenshot from 2024-07-02 16-07-26](https://github.com/TepmarotdanielZ/PID/assets/139426571/32b15de0-6352-4361-b21b-8a1c581d70a0)

DC motor encoders are used for speed control feedback in DC motors where an armature or
rotor with wound wires rotates inside a magnetic field created by a stator. The DC motor
encoder provides a mechanism to measure the speed of the rotor and provide closed loop
feedback to the drive for precise speed control.


