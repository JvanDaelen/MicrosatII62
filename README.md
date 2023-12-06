Hi! This is the repository for the GNCSimulator, a project for the TU Delft Aerospace Engineering Master's course in Microsat Engineering (AE4S10).
Below, you can see the program flow/architecture of the simulator. Each element of the flow will have its own function, which can be called by the overarching simulator.

![](https://github.com/JvanDaelen/MicrosatII62/blob/main/Images/GNCSimulatorFlow.png?raw=true)

The first element is the navigation, which determines the absolute and relative states of the spacecraft, as observed by the sensors.

The second element is the guidance, which uses the Clohessy-Wiltshire equations to determine the desired current state of the spacecraft in order to rendez-vous.

The third element is the control, which determines the error between the observed and desired state and what forces are needed to correct it.

The last element is the dynamics, which uses the current state of the spacecraft and the control forces to determine the next state of the spacecraft.
