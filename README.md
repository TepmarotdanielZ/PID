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

## 3. PID Controller

we will dive into the proportional, integral and derivative as three separate equations, then add
them together to create the output, but first, we need to talk about the user input values in the
PID controller… the gain settings (P-Gain, I-Gain and D-Gain). Gain is the term used for
“multiplication factor”. By adjusting the gain settings (or multiplication factor) of the
proportional, the integral and the derivative, the user can control how much effect the PID
controller has on the output, and how the controller will react to different changes in the process
value.[6

## 4. Proportional Math

The Proportional is calculated by multiplying the P-Gain by the error ( Eq. 4.2 ). The purpose
of the proportional, is to have a large immediate reaction on the output to bring the process
value close to the set point. As the error becomes less, the influence of the proportional value
on the output becomes less.

   The Proportional math looks like this:
   
$$
   𝑝 = \text {𝑝𝑟𝑜𝑝𝑜𝑟𝑡𝑖𝑜𝑛𝑎𝑙}; 𝑘𝑃 = 𝑝𝑟𝑜𝑝𝑜𝑟𝑡𝑖𝑜𝑛𝑎𝑙 𝑔𝑎𝑖𝑛
$$

$$
   𝑆𝑃 = 𝑠𝑒𝑡 𝑝𝑜𝑖𝑛𝑡; 𝑃𝑉 = 𝑝𝑟𝑜𝑐𝑒𝑠𝑠 𝑣𝑎𝑙𝑢𝑒; 𝐸𝑟𝑟 = 𝑒𝑟𝑟𝑜𝑟
$$

$$
   𝐸𝑟𝑟 = 𝑆𝑃 − 𝑃𝑉
$$

$$
   𝑃 = 𝑘𝑃 × 𝐸𝑟𝑟
$$

$$
   ( Eq. 1 )
$$

## 5. Integral Math

The Integral is calculated by multiplying the I-Gain, by the error ( Eq. 4.4 ), then multiplying
this by the cycle time of the controller (how often the controller performs the PID calculation)
and continuously accumulating this value as the “total integral”.
Explained a little further, every time the controller performs the PID calculation (example of a
cycle time in a loop of coding), the new calculated integral value, is added to the integral total.
The integral will normally not have as much immediate influence on the output as the
proportional, but because the integral is continuously accumulating overtime, the longer it takes
for the process value to reach the set point, the more effect the integral will have on the output.
      

And the Integral math:

$$
I = \text{Integral}; \quad K_i = \text{integral gain}
$$

$$
dt = \text{cycle time of the controller}; \quad I_t = \text{integral total}
$$

$$
I = k_I \times \text{Err} \times dt
$$

$$
I_t = I_t + I
$$


## 6. Derivative Math

The derivative is calculated by multiplying the D-Gain by the ramp rate of the process value
( Eq. 1 ). The purpose of the derivative is to “predict” where the process value is going, and
bias the output in the opposite direction of the proportional and integral, to hopefully prevent
the controller from over-shooting the set point if the ramp rate is too fast.
Explained a bit simpler, if the process value is approaching the set point to fast, the derivative
will limit the output to prevent the process value from overshooting the set point.


The Derivative Math:

$$
𝐷 = 𝑑𝑒𝑟𝑖𝑣𝑎𝑡𝑖𝑜𝑛; 𝑘𝐷 = 𝑑𝑒𝑟𝑖𝑣𝑎𝑡𝑖𝑣𝑒 𝑔𝑎𝑖𝑛;
$$

$$
𝑑𝑡 = 𝑐𝑦𝑐𝑙𝑒 𝑡𝑖𝑚𝑒 𝑜𝑓 𝑡ℎ𝑒 𝑐𝑜𝑛𝑡𝑟𝑜𝑙𝑙𝑒𝑟; 𝑝𝐸𝑟𝑟 = 𝑝𝑟𝑒𝑣𝑖𝑜𝑢𝑠 𝑒𝑟𝑟𝑜𝑟
$$

$$
𝐷 = 𝑘𝐷 × 𝐸𝑟𝑟 − 𝑝𝐸𝑟𝑟 / dt
$$





