// New file: Elbow.java (mirroring Arm.java, which was previously Wrist.java)

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.*;

public class Elbow extends SubsystemBase {
    private final TalonFXS elbowMotor;
    private final CANcoder elbowEncoder;
    private final PositionVoltage positionRequest;

    // Current target position in degrees
    private double targetPositionDegrees = 0.0;

    public Elbow() {
        elbowMotor = new TalonFXS(kElbowMotorId, kCANBus.getName());
        elbowEncoder = new CANcoder(kElbowEncoderId, kCANBus.getName());

        // Apply encoder config (absolute range is implicit 0–1)
        elbowEncoder.getConfigurator().apply(elbowEncoderConfigs);

        // Motor configuration
        TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();     

        motorConfig.ExternalFeedback.FeedbackRemoteSensorID = kElbowEncoderId;
        motorConfig.ExternalFeedback.ExternalFeedbackSensorSource =
            ExternalFeedbackSensorSourceValue.RemoteCANcoder;

        motorConfig.ExternalFeedback.SensorToMechanismRatio = 1.0;
        motorConfig.ExternalFeedback.RotorToSensorRatio = kElbowGearRatio;

        motorConfig.Slot0 = elbowPositionGains;
        motorConfig.CurrentLimits = elbowCurrentLimits;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // comment this line out if you want CounterClockwise = positive
        motorConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

        // Set motor type for brushless Minion motor
        motorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // Enable soft limits (motor rotations, post-scaling = mechanism rotations)
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kElbowMaxPosition.in(Degrees) / 360.0;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kElbowMinPosition.in(Degrees) / 360.0;

        // Optional: Voltage compensation for consistent performance
        motorConfig.Voltage.PeakForwardVoltage = 12.0;
        motorConfig.Voltage.PeakReverseVoltage = -12.0;
        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.20;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.20;

        elbowMotor.getConfigurator().apply(motorConfig);
        targetPositionDegrees = getPositionDegrees();

        positionRequest = new PositionVoltage(0).withSlot(0);
    }

    public Command setPositionCommand(double degrees) {
        return runOnce(() -> setTargetPosition(degrees));
    }

    private void setTargetPosition(double degrees) {
        // Clamp to safe range
        targetPositionDegrees = MathUtil.clamp(degrees, kElbowMinPosition.in(Degrees), kElbowMaxPosition.in(Degrees));
        //System.out.println("Setting elbow target to " + targetPositionDegrees + " degrees");
    }

    @Override
    public void periodic() {
        // Target in mechanism rotations
        double motorRotations = targetPositionDegrees / 360.0;

        elbowMotor.setControl(positionRequest.withPosition(motorRotations));
        SmartDashboard.putNumber("Elbow Target Degrees", targetPositionDegrees);
        SmartDashboard.putNumber("Elbow Current Degrees", getPositionDegrees());

        // Optional debug: current and velocity
        SmartDashboard.putNumber("Elbow Motor Current", elbowMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elbow Velocity Degrees/Sec", elbowMotor.getVelocity().getValueAsDouble() * 360.0);
    }

    // Absolute elbow position in degrees
    public double getPositionDegrees() {
        return elbowMotor.getPosition().getValueAsDouble() * 360.0;
    }

    // Check if at target within tolerance
    public boolean isAtPosition(double toleranceDegrees) {
        return Math.abs(getPositionDegrees() - targetPositionDegrees) <= toleranceDegrees;
    }

    @Override
    public void simulationPeriodic() {
        // Simulate position change towards target (simple model)
        double current = getPositionDegrees();
        double delta = (targetPositionDegrees - current) * 0.1; // Simulate movement
        // Update simulated position (in real, use Phoenix sim setup; placeholder for now)
    }
}