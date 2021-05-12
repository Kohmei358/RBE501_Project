Please look at this page in github. It looks much nicer here:
https://github.com/Kohmei358/RBE501_Project

<!-- ABOUT THE PROJECT -->
[![Product Name Screen Shot][product-screenshot]](https://youtu.be/84nr0pFNmIE)

In this project, we have developed a simulated model of the quadruped and dynamic analysis of legs of quadruped is done. Based on the analysis, control techniques were designed for the quadruped, which enables us to achieve the desired gaits. Simulation of the model was done using Simscape Multibody along with MATLAB.

### Goals
* Initial Design and Configuration
* Frame assignment and Kinematics computation
* SolidWorks Design and Importing to Simscape Multibody
* Dynamics Formulation and Trajectory PLanning
* Generating Torque Profiles
* Gait Testing


**Watch a video of our [final presentation](https://youtu.be/84nr0pFNmIE) on Youtube**

**Download our [final submission doc](https://docs.google.com/document/d/1isP2y5EyYgsyj1HRhtxLBZyQ3JKt5DbxevlvPPygO1I/edit?usp=sharing)**

### Built With

* Solidworks
* MATLAB
* Simscape Multibody

<!-- GETTING STARTED -->
## Getting Started

1. Install [MATLAB](https://www.oracle.com/technetwork/java/javase/downloads/index.html)
2. Install [Simulink Multibody](https://www.oracle.com/technetwork/java/javase/downloads/index.html)
3. Install [Robotics Toolbox by Peter Corke](https://gluonhq.com/products/scene-builder/)
4. Clone this repo
   ```sh
   git clone https://github.com/Kohmei358/RBE501_Project
   ```
4.Open and run (Green Play Button) thefile "Assemble1_DataFile1.m"

## Testing Gaits in SIMULINK

1. To simulate the walking gait, run the file BetterGait.slx
2. To simulate the galloping gait, run the file CustomOne.slx

## Testing Lagrange Euler

1. To generate the lagrange equations, run the file Lagrange_Euler_dynamics/Lagrange_Euler_main.m

## Testing Dynmaics + Torque Profile Generation

1. For Newton-Euler Torque profile generation, Please Open ProjectNE.m file and Run the code and observe the joint torque profiles.

<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* Proffessor **Loris Fichera**
* **Neet Mehulkumar mehta** [neetmehta](https://github.com/neetmehta)
* **Kohmei Kadoya.** [@Kohmei358](https://github.com/Kohmei358)
* **Sumanth varma pericherla** [MaverickLynch] (https://github.com/MaverickLynch)
* **Kishor Sabarish Ganapathy subramanian** [kishorsabarishg] (https://github.com/kishorsabarishg)

[product-screenshot]: RBE501Poster.jpg
