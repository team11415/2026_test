// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Optional;
import java.util.Arrays;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private NetworkTable limelightaTable;
  private NetworkTable limelightbTable;
  private NetworkTableEntry throttleEntry;
  private final Field2d m_field = new Field2d();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getOdometryThread().setThreadPriority(99);
  
    SignalLogger.start();

    limelightaTable = NetworkTableInstance.getDefault().getTable("limelight-a");
    throttleEntry = limelightaTable.getEntry("throttle_set");
    SmartDashboard.putData("Field", m_field); // Publishes under /SmartDashboard/Field
  }

  @Override 
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    //Pose2d currentPose = m_robotContainer.drivetrain.getState().Pose; //e.g., from DrivetrainSubsystem odometry
    //field.setRobotPose(currentPose);

    double tv = limelightaTable.getEntry("tv").getDouble(0.0);
    double[] botpose = limelightaTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    SmartDashboard.putNumber("Direct LL TV", tv);
    SmartDashboard.putString("Direct LL Botpose", java.util.Arrays.toString(botpose));


    m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
                  // TEMPORARY TEST: Force robot to center of field
                  //m_field.setRobotPose(new Pose2d(8.0, 4.0, new Rotation2d()));
    
    SmartDashboard.putString("PoseDebug", m_robotContainer.drivetrain.getState().Pose.toString());
    
    //field.getObject("traj").setTrajectory(yourTrajectory); // set a trajectory object

    if (DriverStation.isDisabled()) {
      throttleEntry.setNumber(100);  // High throttle for cooling when disabled
    } else {
      throttleEntry.setNumber(0); // Full performance when enabled
    }
    // Monitor temperature (may be inaccurate during throttling)
    double cpuTemp = limelightaTable.getEntry("cpu_temp").getDouble(0.0);
    //System.out.println("LL4 CPU Temp: " + cpuTemp + "*C");

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.limelightA.setAlliance(
      DriverStation.getAlliance().orElse(Alliance.Blue)
    );
        m_robotContainer.limelightB.setAlliance(
        DriverStation.getAlliance().orElse(Alliance.Blue)
    );
    //m_robotContainer.drivetrain.setForwardHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? 0 : 180));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}