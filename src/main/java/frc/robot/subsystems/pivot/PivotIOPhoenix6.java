package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.config.PortConfiguration;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonFXFactory;

import static frc.robot.subsystems.pivot.PivotSubsystem.FORWARD_SOFT_LIMIT_DEGREES;
import static frc.robot.subsystems.pivot.PivotSubsystem.REVERSE_SOFT_LIMIT_DEGREES;

public class PivotIOPhoenix6 implements PivotIO {
    private final TalonFX pivot;
    private static final double PIVOT_GEAR_RATIO = (36.0 / 9.0) * (40.0 / 14.0) * (64.0 / 12.0);
    private static final double MOTOR_ROTATIONS_PER_DEGREE = PIVOT_GEAR_RATIO / 360.0;
    private static final double DEGREES_PER_ROTATION = 1.0 / MOTOR_ROTATIONS_PER_DEGREE;

    private final TalonFXConfiguration configuration = new TalonFXConfiguration();

    private final PositionDutyCycle request = new PositionDutyCycle(0, 0, true, 0.028, 0, false, false, false);

    public PivotIOPhoenix6(PortConfiguration ports) {
        pivot = TalonFXFactory.createDefaultTalon(ports.pivotID, false);

        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configuration.CurrentLimits.SupplyCurrentLimit = 20.0;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = 80;
        configuration.Audio.BeepOnConfig = false;

        configuration.Slot0.kP = 0.3;
        configuration.Slot0.kI = 0;
        configuration.Slot0.kD = 0;
        configuration.Slot0.kV = 0;
        configuration.Slot0.kA = 0;
        configuration.Slot0.kS = 0;

        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                FORWARD_SOFT_LIMIT_DEGREES * MOTOR_ROTATIONS_PER_DEGREE;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                REVERSE_SOFT_LIMIT_DEGREES * MOTOR_ROTATIONS_PER_DEGREE;

        // Start with soft limits disabled.  Enable them when homing.
        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;

        Phoenix6Util.applyAndCheckConfiguration(pivot, configuration);
    }

    @Override
    public void setSetpointInDegrees(double setpointInDegrees) {
        pivot.setControl(request.withPosition(degreesToMotorRotations(setpointInDegrees)));
    }

    double degreesToMotorRotations(double degrees) {
        return degrees * MOTOR_ROTATIONS_PER_DEGREE;
    }

    @Override
    public void setHomingPosition(double position) {
        pivot.setPosition(degreesToMotorRotations(position));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                        pivot.getMotorVoltage(),
                        pivot.getSupplyCurrent(),
                        pivot.getDeviceTemp(),
                        pivot.getPosition(),
                        pivot.getVelocity())
                .isOK();
        inputs.pivotVoltage = pivot.getMotorVoltage().getValueAsDouble();
        inputs.pivotCurrent = pivot.getSupplyCurrent().getValueAsDouble();
        inputs.pivotTemperature = pivot.getDeviceTemp().getValueAsDouble();
        inputs.pivotPositionDegrees = pivot.getPosition().getValueAsDouble() * DEGREES_PER_ROTATION;
        inputs.pivotVelocityDegrees = pivot.getVelocity().getValueAsDouble() * DEGREES_PER_ROTATION;
    }

    @Override
    public void setPercentage(double percentage) {
        pivot.set(percentage);
    }
}
