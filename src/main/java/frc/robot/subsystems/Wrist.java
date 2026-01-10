package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase {
    private final TalonFXS wristMotor;
    private final CANcoder wristEncoder;
    private final PositionVoltage positionRequest;

    // Current target position in degrees
    private double targetPositionDegrees = 90.0;

    public Wrist() {
        wristMotor = new TalonFXS(kWristMotorId, kCANBus.getName());
        wristEncoder = new CANcoder(kWristEncoderId, kCANBus.getName());

        // Apply encoder config (absolute range is implicit 0–1)
        wristEncoder.getConfigurator().apply(wristEncoderConfigs);

        // Motor configuration
        TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();        

        motorConfig.ExternalFeedback.FeedbackRemoteSensorID = kWristEncoderId;
        motorConfig.ExternalFeedback.ExternalFeedbackSensorSource =
            ExternalFeedbackSensorSourceValue.RemoteCANcoder;

        motorConfig.ExternalFeedback.SensorToMechanismRatio = 1.0;
        motorConfig.ExternalFeedback.RotorToSensorRatio = kWristGearRatio;

        motorConfig.Slot0 = wristPositionGains;
        motorConfig.CurrentLimits = wristCurrentLimits;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Set motor type for Minion
        motorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // Enable soft limits (motor rotations, post-scaling = mechanism rotations)
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kWristMaxPosition.in(Degrees) / 360.0;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kWristMinPosition.in(Degrees) / 360.0;

        // Optional: Voltage compensation for consistent performance
        motorConfig.Voltage.PeakForwardVoltage = 12.0;
        motorConfig.Voltage.PeakReverseVoltage = -12.0;
        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.10;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.10;

        wristMotor.getConfigurator().apply(motorConfig);
        targetPositionDegrees = getPositionDegrees();

        positionRequest = new PositionVoltage(0).withSlot(0);
    }

    public Command setPositionCommand(double degrees) {
        return runOnce(() -> setTargetPosition(degrees));
    }

    private void setTargetPosition(double degrees) {
        // Clamp to safe range
        targetPositionDegrees = MathUtil.clamp(degrees, kWristMinPosition.in(Degrees), kWristMaxPosition.in(Degrees));
        //System.out.println("Setting wrist target to " + targetPositionDegrees + " degrees");
    }

    @Override
    public void periodic() {
        // Target in mechanism rotations
        double motorRotations = targetPositionDegrees / 360.0;

        wristMotor.setControl(positionRequest.withPosition(motorRotations));
        SmartDashboard.putNumber("Wrist Target Degrees", targetPositionDegrees);
        SmartDashboard.putNumber("Wrist Current Degrees", getPositionDegrees());

        // Optional debug: current and velocity
        SmartDashboard.putNumber("Wrist Motor Current", wristMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Wrist Velocity Degrees/Sec", wristMotor.getVelocity().getValueAsDouble() * 360.0);
    }

    // Absolute wrist position in degrees
    public double getPositionDegrees() {
        return wristMotor.getPosition().getValueAsDouble() * 360.0;
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