// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Vision;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Util.RectanglePoseArea;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll;
  private NetworkTable limelightTable;
  private Boolean enable = false;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
  
        /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain, String limelightName) {
    this.drivetrain = drivetrain;
    this.ll = limelightName;
    this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
  }
@Override
public void periodic() {
    SmartDashboard.putBoolean("LL Enable", enable);
  if (enable) {
    // Get data directly from NetworkTables instead of JSON parsing
    double tv = limelightTable.getEntry("tv").getDouble(0.0);
    SmartDashboard.putNumber("LL TV", tv);
    
    if (tv == 1.0) {  // Target visible
      double[] botpose_wpiblue = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[18]);
      SmartDashboard.putString("LL Botpose", java.util.Arrays.toString(botpose_wpiblue));
      SmartDashboard.putNumber("LL Botpose Length", botpose_wpiblue.length);
       
      if (botpose_wpiblue.length >= 6) {
        botpose = new Pose2d(
          botpose_wpiblue[0],
          botpose_wpiblue[1],
          Rotation2d.fromDegrees(botpose_wpiblue[5])
        );
        SmartDashboard.putString("LL Pose2d", botpose.toString());
        
        if (field.isPoseWithinArea(botpose)) {
          SmartDashboard.putBoolean("LL In Field", true);
          
          double targetDistance = Math.sqrt(
            botpose_wpiblue[0] * botpose_wpiblue[0] + 
            botpose_wpiblue[1] * botpose_wpiblue[1]
          );
          double confidence = 1 - ((targetDistance - 1) / 6);
          
          double currentDistance = drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation());
          SmartDashboard.putNumber("LL Distance Check", currentDistance);
          SmartDashboard.putBoolean("LL Trust", trust);
          
          if (currentDistance < 0.5 || trust) {
            SmartDashboard.putBoolean("LL Applied", true);
            
            double latency = limelightTable.getEntry("tl").getDouble(0.0) + 
                           limelightTable.getEntry("cl").getDouble(0.0);
            
            drivetrain.addVisionMeasurement(
              botpose,
              Timer.getFPGATimestamp() - (latency / 1000.0),
              VecBuilder.fill(confidence, confidence, .01)
            );
          } else {
            SmartDashboard.putBoolean("LL Applied", false);
            distanceError++;
            SmartDashboard.putNumber("Limelight Error", distanceError);
          }
        } else {
          SmartDashboard.putBoolean("LL In Field", false);
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
      }
    }
  }
}
  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }
  public void useLimelight(boolean enable) {
    this.enable = enable;
  }
  public void trustLL(boolean trust) {
    this.trust = trust;
  }
}