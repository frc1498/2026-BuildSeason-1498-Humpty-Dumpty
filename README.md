# 2026-BuildSeason-1498

[![CI](https://github.com/frc1498/2026-BuildSeason-1498/actions/workflows/main.yml/badge.svg)](https://github.com/frc1498/2026-BuildSeason-1498/actions/workflows/main.yml)

Code for Team 1498's 2026 robot, Aurora Zwei. <br>
[ADD ROBOT PICTURE HERE]

## General

01/17/26 - Proved out the functionality of DogLog.  Will start integrating the logging into the subsystems. <br>
01/17/26 - Started creating the code framework with subsystems, configurations, and constants. <br>

## Planned Subsystems

### Drivetrain
01/17/26 - CTRE Swerve Template generated. <br>
01/17/26 - Added the PathPlanner base code into the drive subsystem. <br>
01/29/26 - Auton selection is roughly in place.  The code will crash if there are no autons for the specified alliance. Allow an empty command.<br>
01/29/26 - I don't like the auton selector, but I don't have a better solution in place.  At the least, maybe simplify the code for the Selector class.

#### *To-Do*
- Drive the robot on the floor to determine gains.
- Add code to drive to a set position during TeleOp.

### Vision
01/21/26 - Created the vision subsystem. <br>
01/21/26 - Added in Limelight functionality.  The code is untested, but it should integrate the megaTag2 estimate into the drivetrain. <br>
01/21/26 - Started adding in Photonvision functionality.  Still need to look at example code, integrate it into the subsystem, and test.
#### *Photonvision Notes*
From looking at the Photonvision documentation and examples, the general process to getting a pose estimate from a camera is:
- Read all currently unread results from the camera.
- For each result, estimate the pose.  If the pose is empty (i.e. couldn't be estimated), estimate the lowest ambiguity pose.
- From the pose, calculate the Std. Dev to pass along with the pose to the drivetrain.
- Convert the 3d pose to a 2d pose.
- Send the 2d pose, timestamp, and Std. Dev into the drivetrain. <br>

This needs to be done for both photonvision cameras.
#### *To-Do*
- Measure the robot-relative position of the Limelight on the robot.
- Test the limelight functionality.
- Finish adding in photonvision functionality.
- Finish the list of variables to track in NetworkTables.
- List variables to log with DogLog.

### LED
Every year, the team wants to add LEDs to the robot for signalling, but it's always a low priority. <br>
Maybe this year will be the year.
#### *To-Do*
- Define parameters for each state (color, pattern, etc?).
- Define maximum amount of LEDs (including CANdle and LED strip).
- Create configuration settings.
- Create base commands for setting the mode.
- Create command factories for each state.
- List variables to track in NetworkTables.
- List variables to log with DogLog.

### Intake
An intake for picking up game pieces. <br>
Through bumper? Over the bumper? We'll find out.
#### *To-Do*
- Everything

### Hopper
Some system for agitating the game pieces and moving them up into the shooter.
#### *To-Do*
- Everything

### Shooter
A shooter that includes a turret, hood adjuster, and flywheel. <br>
Maybe more (definitely more), but this is just a draft...
#### *To-Do*
- Everything
### Climber
Probably some kind of hook or clamp to latch on the climb bars.
#### *To-Do*
- Everything
