// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

            // This declares a public class called "Robot" that extends (inherits from) "TimedRobot", which runs code in loops at fixed intervals.
public class Robot extends TimedRobot {
            // This is a private field (variable) to store a command for autonomous mode. It's like a placeholder for an action the robot will do automatically.
  private Command m_autonomousCommand;

            // This is a private final field for the RobotContainer, which  holds all the robot's subsystems and commands.
  private final RobotContainer m_robotContainer;

            // This is a private final boolean flag to decide whether to use Limelight. It's set to false, so vision code is disabled by default.
  private final boolean kUseLimelight = false;

            // This is the constructor for the Robot class. It runs when a new Robot object is created and initializes the RobotContainer.
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

            // This overrides the robotPeriodic method from TimedRobot. It runs repeatedly (about every 20ms) no matter what mode the robot is in.
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    
            // This checks if the Limelight flag is true. If so, it runs vision-related code; otherwise, it skips this block.
     if (kUseLimelight) {
            // This gets the current state of the drivetrain (like the robot's position and speed) from the RobotContainer.
      var driveState = m_robotContainer.drivetrain.getState();
            // This calculates the robot's heading (direction) in degrees from its pose (position and orientation).
      double headingDeg = driveState.Pose.getRotation().getDegrees();
            // This converts the robot's rotational speed from radians per second to rotations per second.
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            // This sets the robot's orientation in the Limelight system, using the heading and zeros for other values (like pitch, roll).
      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            // This gets a pose estimate from Limelight using MegaTag2 (a vision processing mode) for the blue alliance field.
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            // This checks if the measurement is valid, has at least one tag detected, and the robot isn't spinning too fast (less than 2 rotations per second).
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                // If conditions are met, this adds the vision measurement to the drivetrain's pose estimator for better accuracy.
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

            // This overrides disabledInit, which runs once when the robot enters disabled mode (not moving). Here, it's empty, so nothing happens.
  @Override
  public void disabledInit() {}

            // This overrides disabledPeriodic, which runs repeatedly in disabled mode. Empty, so no actions.
  @Override
  public void disabledPeriodic() {}

            // This overrides disabledExit, which runs once when leaving disabled mode. Empty.
  @Override
  public void disabledExit() {}

            // This overrides autonomousInit, which runs once when entering autonomous mode.
  @Override
  public void autonomousInit() {
            // This gets the autonomous command from the RobotContainer.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

            // This checks if the command exists, and if so, schedules it to run.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

            // This overrides autonomousPeriodic, which runs repeatedly in autonomous mode. Empty.
  @Override
  public void autonomousPeriodic() {}

            // This overrides autonomousExit, which runs once when leaving autonomous mode. Empty.
  @Override
  public void autonomousExit() {}

            // This overrides teleopInit, which runs once when entering teleoperated mode.
  @Override
  public void teleopInit() {
            // This checks if an autonomous command is still running and cancels it if so, to switch to driver control.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

            // This overrides teleopPeriodic, which runs repeatedly in teleop mode. Empty.
  @Override
  public void teleopPeriodic() {}

            // This overrides teleopExit, which runs once when leaving teleop mode. Empty.
  @Override
  public void teleopExit() {}

            // This overrides testInit, which runs once when entering test mode (for debugging).
  @Override
  public void testInit() {
            // This cancels all running commands to start fresh in test mode.
    CommandScheduler.getInstance().cancelAll();
  }

            // This overrides testPeriodic, which runs repeatedly in test mode. Empty.
  @Override
  public void testPeriodic() {}

            // This overrides testExit, which runs once when leaving test mode. Empty.
  @Override
  public void testExit() {}

            // This overrides simulationPeriodic, which runs repeatedly in simulation mode (for testing without hardware). Empty.
  @Override
  public void simulationPeriodic() {}
}
