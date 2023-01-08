# ROV_8_ThrusterControl
Aim of this project is to control 8 thrusters of an AUV using PID and LQR controllers

Checkout the Theory.md file inside the MCS folder. It has all the content required to understand this project. 

The Non-Linear PID model(.slx file) is provided which explains the control system with PID controller.

For the LQR controller based control system, there are three files provided. 

One is the '.m' file that is required to be executed to implement the '.slx' files. 

'LQR1.slx' uses a basic LQR controller without any noise added.

'LQR2.slx' involves white noise(check both the models: commented and uncommented in this file)
