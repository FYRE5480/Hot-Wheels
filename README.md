# Hot Wheels
### Some profound text goes here -

This is a template repository meant to be used for a new year's robot code. It can be easily used to set up a fully capable drive base to build off of. It contains: 
* our custom swerve code
* a dashboard setup ([Elastic](https://frc-elastic.gitbook.io/docs))
* the ability to load and follow [Choreo](https://choreo.autos) trajectories
* integration with our custom vision processing software, [Astrolabe](https://github.com/MaxedPC08/Astrolabe)

Before using it is **crucial** that you ensure that all constants in `Constants.java` are correct, the template currently contains placeholders. Values will obviously vary from robot to robot. The most important things to check are the PID values, feedforward values, and the absolute offsets. You will also need to update WPI and all of the libraries that the code uses to the current year. Most of this can be done by using the `WPILib: Import a project` feature. 

Absolute offsets are just the value that your absolute swerve encoders read when the wheels are pointing at 0. They are listed in the order that the modules are in the array of modules. The easiest way to find these values is to have each module print its value when the wheels are straight and record their values.

## PLEASE PLEASE PLEASE update this repository when used for a new season to make sure that it stays up to date and usable for future seasons. 
Do not update it with any code other than the swerve, auto and controller features, as it should remain a framework.

Thanks :)
