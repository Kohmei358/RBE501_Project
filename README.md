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

**See more [screenshots](https://drive.google.com/drive/folders/1IVsE6aBYgut-1ja_xGQBNLKaQGX4bQh1?usp=sharing) of our app**

**Download our [final submission doc](https://docs.google.com/document/d/1isP2y5EyYgsyj1HRhtxLBZyQ3JKt5DbxevlvPPygO1I/edit?usp=sharing) which contains information on our Scrum process**

### Built With

* Solidworks
* MATLAB
* Simscape Multibody

<!-- GETTING STARTED -->
## Getting Started

You can download a pre-compiled JAR file to run as a demo or clone the project if you would like to edit it.

### Download Compiled Program
Make sure you have [Java 8](https://www.oracle.com/technetwork/java/javase/downloads/index.html) installed! **This program does not run on any new versions of Java.**

1. Download and unzip this folder: [Download JAR](https://drive.google.com/file/d/1WHYaf3kWZZlwsQCP4EVbTBOdXofKgJuf/view?usp=sharing)
2. Double click the file ```CS3733-C21-Team-U-Project-BWApp.jar```
3. If this does not work, run this command in your command line
   ```sh
   java -jar /Path/To/Downloaded/Folder/CS3733-C21-Team-U-Project-BWApp.jar
   ```
   
Please use the buttons on the bottom left of the starting screen to simulate logging in if you don't know any usernames / passwords.

### Download and Run the Source Code

1. Install [Java SE Development Kit 8u281 (Java SE JDK 8)](https://www.oracle.com/technetwork/java/javase/downloads/index.html)
2. Install [Gluon Scene Builder 8.5.0 for Java 8](https://gluonhq.com/products/scene-builder/)
3. Install [Apache Derby 10.14.2.0 for Java 8](https://db.apache.org/derby/derby_downloads.html#For+Java+8+and+Higher) 
Download the bin distribution – the first link ending in .zip. Then double-click on the zip file.
4. Clone this repo
   ```sh
   git clone https://github.com/Kohmei358/BrighamWomens-Hospital-App
   ```
3. Install Dependencies Using Gradle
   ```sh
   gradle build
   ```
4. Run the ```main.java``` or ```App.java``` file inside ```/src/main/java/edu/wpi/u/```


<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* Proffessor **Loris Fichera*
* **Kohmei K.**	Lead Software Engineer [@Kohmei358](https://github.com/Kohmei358)
* **Neet Mehulkumar mehta** [@beast-nev](https://github.com/beast-nev)
* **Sumanth varma pericherla** [@kslokhandwala2022](https://github.com/kslokhandwala2022)
* **Kishor Sabarish Ganapathy subramanian** [@kslokhandwala2022](https://github.com/kslokhandwala2022)

## Disclaimer
This was forked so that I could link this code in [my website](https://kohmeik.com). Please do not reference any of the code or UX design if you are currently enrolled in Software Engineering! The learning experience is so much better if you do not look at examples of previous work and your team will end up with a better final product if you brainstorm based on your surveys and interviews.

[product-screenshot]: RBE501Poster.jpg
