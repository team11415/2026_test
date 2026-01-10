package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax; // Import for the main SPARK MAX controller class.
import com.revrobotics.RelativeEncoder; // Import for the relative encoder (built-in to SPARK MAX).
import com.revrobotics.spark.SparkClosedLoopController; // Import for the closed-loop (PID) controller.
import com.revrobotics.spark.SparkLowLevel.MotorType; // Import for motor type enum (e.g., kBrushless).
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig; // Import for the base configuration object used to set parameters.
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; // Import for idle mode enum (kBrake, kCoast).
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode; // Import for reset mode enum used in configure().
import com.revrobotics.spark.SparkBase.PersistMode; // Import for persist mode enum used in configure() to burn to flash.
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType; // Import for control type enum (e.g., kPosition).
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;

public class Iris extends SubsystemBase {
    // The SparkMax motor controller for the Neo 550 motor, initialized with CAN ID 55 and brushless motor type.
private final SparkMax irisMotor;
    // The built-in relative encoder from the SparkMax, used to track motor rotations.
private final RelativeEncoder encoder;

// The closed-loop controller from SPARK MAX for position control.
private final SparkClosedLoopController closedLoopController;

// Current target position in motor rotations (since no gear reduction).
private double targetPositionRotations = 0.0;

// Constructor for the Iris subsystem. Initializes hardware and configurations.
public Iris() {
    // Initialize SparkMax with CAN ID from constants and specify brushless for Neo 550.
    irisMotor = new SparkMax(kIrisMotorId, MotorType.kBrushless);

// Create a configuration object to set all parameters at once (SPARK MAX API uses this for batch configuration).
   SparkMaxConfig config = new SparkMaxConfig();


    // Create a closed-loop configuration and set PID gains using chaining methods (for default slot 0).
    ClosedLoopConfig clConfig = new ClosedLoopConfig()
        .p(kIrisP)  // Sets proportional gain.
        .i(kIrisI)  // Sets integral gain.
        .d(kIrisD)  // Sets derivative gain.
        // For feedforward, use deprecated pidf if needed, or configure via feedForward for advanced FF.
        // Since kIrisFF = 0.0, omitting or using .pidf(kIrisP, kIrisI, kIrisD, kIrisFF) if compatible.
        .outputRange(-1.0, 1.0);  // Sets output range to limit motor power.

    // Apply the closed-loop config to the main config.
    config.apply(clConfig);

    // Set current limits to prevent overheating/damage (uses method with int amps).
    config.smartCurrentLimit(kIrisCurrentLimit);

    // Set idle mode to brake for holding position (prevents coasting).
    config.idleMode(IdleMode.kBrake);

// Apply the configuration to the motor, resetting safe parameters and persisting to flash (equivalent to burnFlash).
    // ResetMode.kResetSafeParameters: Resets parameters that are safe to change without reboot.
    // PersistMode.kPersistParameters: Saves the config to non-volatile memory so it persists across power cycles.
    irisMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get the relative encoder; default is in rotations (no conversion factor needed since no gearing).
    encoder = irisMotor.getEncoder();

    // Get the closed-loop controller for setting references (e.g., position targets).
    closedLoopController = irisMotor.getClosedLoopController();

    // Initialize target to current position to avoid sudden movement on enable.
    targetPositionRotations = encoder.getPosition();
}

// Returns a command to set the iris to a specific position in rotations.
// Usage: iris.setPositionCommand(0.25).schedule(); but typically bound to buttons.
public Command setPositionCommand(double rotations) {
    return runOnce(() -> setTargetPosition(rotations));
}

// Private method to update the target position. No clamping since no min/max specified in query (add if physical limits exist).
private void setTargetPosition(double rotations) {
    targetPositionRotations = rotations; // Direct set; add clamps if needed (e.g., MathUtil.clamp(rotations, min, max)).
    // System.out.println("Setting iris target to " + targetPositionRotations + " rotations"); // Optional debug print.
}

// Periodic method runs every loop (~20ms). Applies position control and logs data.
@Override
public void periodic() {
    // Set the closed-loop reference to the target position using position control mode (applies PID to reach/hold target).
    // CHANGE: Replaced deprecated setReference with setSetpoint (same signature; use SparkBase.ControlType.kPosition).
    closedLoopController.setReference(targetPositionRotations, SparkBase.ControlType.kPosition);

    // Log target and current position to SmartDashboard for tuning/debugging.
    SmartDashboard.putNumber("Iris Target Rotations", targetPositionRotations);
    SmartDashboard.putNumber("Iris Current Rotations", encoder.getPosition());

    // Optional: Log current and velocity for monitoring.
    SmartDashboard.putNumber("Iris Motor Current", irisMotor.getOutputCurrent()); // Gets the output current (may need verification if method exists; alternatively, use getAppliedOutput() * bus voltage for estimate if not directly available).
    SmartDashboard.putNumber("Iris Velocity RPS", encoder.getVelocity() / 60.0); // getVelocity() returns RPM; divide by 60 for rotations per second (RPS).
}

// Get current position in rotations.
public double getPositionRotations() {
    return encoder.getPosition();
}

// Check if at target within a tolerance (e.g., 0.01 rotations; tune as needed).
public boolean isAtPosition(double tolerance) {
    return Math.abs(getPositionRotations() - targetPositionRotations) <= tolerance;
}

// Simulation periodic for testing in sim environment (simple model).
@Override
public void simulationPeriodic() {
    // Simulate position approaching target (placeholder; use REV Physics Sim for accurate simulation if needed).
    double current = getPositionRotations();
    double delta = (targetPositionRotations - current) * 0.1; // Gradual movement simulation.
    // In real sim, update via physics model (e.g., integrate velocity).
}}

