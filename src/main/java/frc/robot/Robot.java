// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.HootEpilogueBackend;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  @NotLogged
  private Command m_autonomousCommand;
  @NotLogged
  private Command m_limelightCommand;

  private final RobotContainer m_robotContainer;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    .withTimestampReplay()
    .withJoystickReplay();

  /**
  * This function is run when the robot is first started up and should be used for any
  * initialization code.
  */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    Epilogue.configure(config -> {
      config.backend = EpilogueBackend.multi(
        new HootEpilogueBackend(),
        new NTEpilogueBackend(NetworkTableInstance.getDefault())
      );
      config.minimumImportance = Logged.Importance.DEBUG;
    });

    Epilogue.bind(this);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // Port Forwarding for the limelight.
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    // Get the deploy directory for Elastic.
    //WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Set the RoboRIO2 custom brownout voltage.
    RobotController.setBrownoutVoltage(4.5);

    // This code was previously PathfindingCommand.warmupCommand.schedule();  That format has been deprecated, so I'm using this new format.
    // This warmup command should help with the initial loading of autonomous paths.
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /**
  * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
  * that you want ran during disabled, autonomous, teleoperated and test.
  *
  * <p>This runs after the mode specific periodic functions, but before LiveWindow and
  * SmartDashboard integrated updating.
  */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    // Commenting out the HootReplay functionality.
    // m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run(); 
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    // Schedule the command to record the limelight video for auton when the auton period ends.
    m_limelightCommand = m_robotContainer.vision.limelightAutonVideo();
    CommandScheduler.getInstance().schedule(m_limelightCommand.withName("limelightAutonVideo"));
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putString("Is Hub Active?", MatchInfo.getInstance().isCurrentHubActive());
    //SmartDashboard.putString("Next Hub Active?", MatchInfo.getInstance().isNextScoringHubActive(5));
  }

  @Override
  public void teleopExit() {
    // Schedule the command to record the limelight video for teleop when the teleop period ends.
    m_limelightCommand = m_robotContainer.vision.limelightTeleopVideo();
    CommandScheduler.getInstance().schedule(m_limelightCommand.withName("limelightTeleopVideo"));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
