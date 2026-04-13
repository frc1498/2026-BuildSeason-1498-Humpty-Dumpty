# 2026-BuildSeason-1498-B-Side

[![CI](https://github.com/frc1498/2026-BuildSeason-1498/actions/workflows/main.yml/badge.svg)](https://github.com/frc1498/2026-BuildSeason-1498/actions/workflows/main.yml)

Code for Team 1498's 2026 rebuilt robot, Aurora Zwei. <br>
[ADD ROBOT PICTURE HERE]

## General
I propose renaming this robot. <br>
I think it should still follow the team convention - Aurora Zwei - but it needs a better qualifier than '.II' or 'Mark 2'. <br>
The project is named Humpty-Dumpty because the rebuild is what the youths of today call a 'dumper bot', but I have another proposal. <br>
Enter 'Aurora Zwei: B-Side'. <br>
Granted, our team branding has no musical connection at all, but I think it's cleaner than Aurora Zwei: Electric Boogaloo.<br>
Think about it, won't you?

## *To-Do*
- Update the robotToShooter transform to reflect the *new* position of the shooter on the robot, instead of the old turret position.  Either use the center of the shooter (an X of zero, a Y of the back of the robot) or use the center of the robot.  Either way, TEST IT TO MAKE SURE!
- Consider splitting the hood into a new subsystem separate from the shooter.  That would potentially add a lag between the calculated hood angle (from the shooter subsystem) and the desired hood angle (supplied to the hood subsystem).
- Consider de-coupling the target location from the shooter subsystem.  It could be calculated and updated in the MatchInfo file, and commands in Move.java could call the method.  The shooter could then get the value as a Supplier.
- Tune the PID for the aim while moving functionality.  I took those numbers from a previous robot, and I'm assuming the drivetrain is actually very different. I didn't bother to check!
