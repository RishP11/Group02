**Ultrasonic Sensor based Parking Assist System**

*Course:* EE 615 Embedded Systems Lab 

*Group 02 Members*: 
1. Ganesh Panduranga Karamsetty - 210020009
2. Rishabh Pomaje - 210020036

> Description

- The entire system is based on the Tiva C series TM4C123G LaunchPad series evaluation kit. 
- Using ultrasonic sensors, we can measure the distance between two points. 
 
![alt text](Images/exp01.svg)

- As shown in Fig. 1a, the ultrasonic sensor (US) board contains a pair of devices, a Transmitter and a Receiver. 
    - Transmitter sends an Ultrasonic pulse of a small duration. 
    - This pulse then travels on its path until it meets and faces an obstacle. 
    - The obstacle then reflects back this pulse. The receiver waits for the pulse.

- The board interfaces with the microcontroller using 2 communication signal pins, $\texttt{Trig}$ and $\texttt{Echo}$.

- How can we use this to measure the distance to the obstacle?
    1. Send a signal and start the timer. 
    2. Wait for the reflected signal.
    3. Once we get back the signal, check the time elapsed. 
    4. Then using simple equation, 
    $\begin{align}
        \text{distance} (d) = \text{speed} (s) \times \text{time elapsed} (t). 
    \end{align}$
    where,
    d is the distance traversed by the pulse and its reflection (i.e., twice the distance we require). Speed in this case corresponds to the speed of sound, i.e., $\sim 343$ m/s at $20\degree$.
    5. Thus, we get the required distance.

- Our plan:
    - As mentioned previously, we plan to use the TI TM4C1236GH6PM microcontroller to read and operate the two USs. 
    - We will be outputting the distance of the two USs to the PC using serial UART.
    - Along with this as a visual aid, we will blink the on-device LED with a frequency  inversely proportional to the distance to the obstacle. 

> System Block Diagram (Hardware)
![alt text](Images/hardware_bl.svg)

> System Block Diagram (Software)
<!-- ![alt text](Images/exp01.svg) -->
    - Comming soon..
- Materials and Datasheet:
    | Material | Link to Datasheet |
    | -------- | ----------------- |
    | Tiva C series TM4C123G <br> LaunchPad evaluation kit | https://www.ti.com/product/TM4C123GH6PM |
    | HC-SR04 | https://robu.in/wp-content/uploads/2014/08/edited_HC-SR04-User-Manual-1.pdf

- Timeline:
    | Date | Milestone |
    | -------- | ------ |
    | 29/10/2024 | Initial plan formulation |