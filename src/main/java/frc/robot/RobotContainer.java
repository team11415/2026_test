// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Vision.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Iris;
import frc.robot.subsystems.Turret;

        // This declares a public class called RobotContainer, which holds all the robot's subsystems, commands, and bindings (like button mappings).
public class RobotContainer {
        // This sets the maximum speed based on tuned constants, converted to meters per second. It's the top speed the robot can go.
    private double MaxSpeed = Constants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        // This sets the maximum angular (turning) rate to 0.75 rotations per second, converted to radians per second.
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // // Add a speed multiplier for limiting (e.g., 0.5 for 50% speed)
    // private double speedMultiplier = 1.0; // Default to full speed
    // private final double kLimitedMultiplier = 0.2; // Adjust this value for your desired limit (e.g., 0.3 for 30%)

    // NEW: Add a speed multiplier for dynamic limiting (starts at 1.0 for full speed)
    private double speedMultiplier = 1.0;
    private final double limitedSpeedFraction = 0.2; // Adjust this (e.g., 0.3 for 30% of max speed)
    
    /* Setting up bindings for necessary control of the swerve drive platform */
        // This creates a field-centric drive request, where the robot moves relative to the field. It adds a 10% deadband (ignores small inputs) and uses open-loop voltage control.
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        // This creates a brake request to stop the robot by putting wheels in an X formation.
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // This creates a request to point all wheels in a specific direction.
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        // This creates a robot-centric forward drive request (moves relative to the robot's orientation) with open-loop control.
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // This creates a Telemetry object for logging data, using the max speed for scaling.
    private final Telemetry logger = new Telemetry(MaxSpeed);

        // This sets up an Xbox controller on port 0 that can trigger commands.
    private final CommandXboxController joystick = new CommandXboxController(0);
   
       // This initializes the swerve drivetrain using tuned constants.
    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();
    

    /* Path follower */
        // This creates a chooser for selecting autonomous commands (pre-programmed paths) from the dashboard.
    private final SendableChooser<Command> autoChooser;

    private final Wrist wrist = new Wrist();
    private final Elbow elbow = new Elbow();
    private final Iris iris = new Iris();
    public final Limelight limelightA = new Limelight(drivetrain, "limelight-a");
    public final Limelight limelightB = new Limelight(drivetrain, "limelight-b");

        // This is the constructor for RobotContainer. It sets up the auto chooser, puts it on the dashboard, configures button bindings, and warms up PathPlanner to avoid delays.
        public RobotContainer() {
            autoChooser = AutoBuilder.buildAutoChooser("Tests");
            SmartDashboard.putData("Auto Mode", autoChooser);
        
     configureBindings(); 

     // Enable both Limelights
    limelightA.useLimelight(true);
    limelightB.useLimelight(true);
        
    CommandScheduler.getInstance().registerSubsystem(limelightA);
    CommandScheduler.getInstance().registerSubsystem(limelightB);
    
    FollowPathCommand.warmupCommand().schedule();
        }


        // This private method sets up all the button and trigger bindings for the controller and robot modes.
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // This sets the default command for the drivetrain: it reads joystick inputs, applies the speed multiplier (though not shown here—might be applied elsewhere), and drives field-centric.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
                // This binds the idle command to run while the robot is disabled, ignoring disable state to keep motors in neutral mode.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

                // This binds the A button to apply the brake while held.
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // This binds the B button to point wheels based on left joystick direction while held.
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

                // This binds the up POV (directional pad) to drive forward at half speed robot-centric.
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
                // This binds the down POV to drive backward at half speed robot-centric.
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // These bind combinations of back/start buttons with X/Y to run SysId tests for tuning (dynamic and quasistatic forward/reverse).
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // // NEW: Toggle speed limit on right bumper press (hold for limited speed, release for full)
        // joystick.rightBumper().whileTrue(
        //     Commands.runOnce(() -> speedMultiplier = kLimitedMultiplier) // Limit speed while held
        // ).onFalse(
        //     Commands.runOnce(() -> speedMultiplier = 1.0) // Restore full speed on release
        // );

        // NEW: Bind right bumper to toggle speed limit (hold for limited mode)
        //joystick.rightBumper().whileTrue(Commands.run(() -> speedMultiplier = limitedSpeedFraction))
        //                       .onFalse(Commands.runOnce(() -> speedMultiplier = 1.0));
        joystick.rightBumper().whileTrue(Commands.run(() -> speedMultiplier = 1.0))
                                 .onFalse(Commands.runOnce(() -> speedMultiplier = limitedSpeedFraction));

                // This registers the telemetry logger with the drivetrain to send data.
        drivetrain.registerTelemetry(logger::telemeterize);

        // In configureBindings(), add example bindings (e.g., X button to 0°, Y to 120°):
        // joystick.x().onTrue(wrist.setPositionCommand(90)); // Move to 0 degrees on X button press
        // joystick.y().onTrue(wrist.setPositionCommand(135)); // Move to 120 degrees on Y button press
        joystick.x().onTrue(Commands.parallel(
            wrist.setPositionCommand(90),
            elbow.setPositionCommand(0)
        ));
        
        joystick.y().onTrue(Commands.parallel(
            wrist.setPositionCommand(135),
            elbow.setPositionCommand(180)
        ));
        // Bind left and right triggers to iris positions (treat as button press when axis > 0.5)
        //joystick.leftTrigger().onTrue(iris.setPositionCommand(0.04)); // Set to 0.25 rotations on left trigger press
        //joystick.rightTrigger().onTrue(iris.setPositionCommand(0.21)); // Set to 0.75 rotations on right trigger press
        joystick.leftTrigger().onTrue(iris.spinRelativeCommand(-0.5)); // //spins -0.5 rotations on left trigger press
        joystick.rightTrigger().onTrue(iris.spinRelativeCommand(0.5)); //spins +0.5 rotations on right trigger press


// Optionally, set default command to hold current position (but since periodic does that, maybe not needed)

        
    }
        // This registers the telemetry logger with the drivetrain to send data.
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
