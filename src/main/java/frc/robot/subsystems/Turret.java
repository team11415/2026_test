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

public class Turret extends SubsystemBase {
    private final TalonFXS turretMotor;
    private final CANcoder turretEncoder;
    private final PositionVoltage positionRequest;

        // Current target position in degrees
private double targetPositionDegrees = 90.0;

 public Turret() {
        turretMotor = new TalonFXS(kturretMotorId, kCANBus.getName());
        turretEncoder = new CANcoder(kturretEncoderId, kCANBus.getName());

        // Apply encoder config (absolute range is implicit 0–1)
        turretEncoder.getConfigurator().apply(turretEncoderConfigs);

        // Motor configuration
        TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();        

        motorConfig.ExternalFeedback.FeedbackRemoteSensorID = kturretEncoderId;
        motorConfig.ExternalFeedback.ExternalFeedbackSensorSource =
            ExternalFeedbackSensorSourceValue.RemoteCANcoder;

        motorConfig.ExternalFeedback.SensorToMechanismRatio = 6.66667;
        motorConfig.ExternalFeedback.RotorToSensorRatio = kturretGearRatio;

        motorConfig.Slot0 = turretPositionGains;
        motorConfig.CurrentLimits = turretCurrentLimits;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Set motor type for Minion
        motorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        // Enable soft limits (motor rotations, post-scaling = mechanism rotations)
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kturretMaxPosition.in(Degrees) / 360.0;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kturretMinPosition.in(Degrees) / 360.0;

        // Optional: Voltage compensation for consistent performance
        motorConfig.Voltage.PeakForwardVoltage = 12.0;
        motorConfig.Voltage.PeakReverseVoltage = -12.0;
        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.10;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.10;

        turretMotor.getConfigurator().apply(motorConfig);
        targetPositionDegrees = getPositionDegrees();

        positionRequest = new PositionVoltage(0).withSlot(0);
    }
public Command setPositionCommand(double degrees) {
        return runOnce(() -> setTargetPosition(degrees));
    }
    
    private void setTargetPosition(double degrees) {
        // Clamp to safe range
        targetPositionDegrees = MathUtil.clamp(degrees, kturretMinPosition.in(Degrees), kturretMaxPosition.in(Degrees));
    }
    
    @Override
    public void periodic() {
        // Target in mechanism rotations
        double motorRotations = targetPositionDegrees / 360.0;

        turretMotor.setControl(positionRequest.withPosition(motorRotations));
        SmartDashboard.putNumber("Turret Target Degrees", targetPositionDegrees);
        SmartDashboard.putNumber("Turret Current Degrees", getPositionDegrees());

        // Optional debug: current and velocity
        SmartDashboard.putNumber("Turret Motor Current", turretMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Turret Velocity Degrees/Sec", turretMotor.getVelocity().getValueAsDouble() * 360.0);
    }
    
    // Absolute turret position in degrees
    public double getPositionDegrees() {
        return turretMotor.getPosition().getValueAsDouble() * 360.0;
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
