package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.config.PortConfiguration;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonFXFactory;

public class AmpBarIOPhoenix6 implements AmpBarIO {
    public final TalonFX ampBar;
    private final TalonFXConfiguration configuration = new TalonFXConfiguration();

    private static final double AMP_BAR_GEAR_RATIO = (24.0 / 14.0) * (40.0 / 18.0) * (40.0 / 24.0) * (49.0 / 19.0);
    private static final double MOTOR_ROTATIONS_PER_DEGREE = AMP_BAR_GEAR_RATIO / 360.0;
    private static final double DEGREES_PER_ROTATION = 1.0 / MOTOR_ROTATIONS_PER_DEGREE;
    private static final double FORWARD_SOFT_LIMIT_DEGREES = 180.0;
    private static final double REVERSE_SOFT_LIMIT_DEGREES = -70.0 * (18.0 / 49.0) * (24.0 / 40.0);

    private final PositionDutyCycle ampBarRequest =
            new PositionDutyCycle(0.0).withFeedForward(0.0).withUpdateFreqHz(0);

    public AmpBarIOPhoenix6(PortConfiguration ports) {
        ampBar = TalonFXFactory.createDefaultTalon(ports.ampBarID);
        configuration.MotorOutput.PeakForwardDutyCycle = 0.25;
        configuration.MotorOutput.PeakReverseDutyCycle = -0.25;
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configuration.CurrentLimits.SupplyCurrentLimit = 20.0;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = 20.0;

        configuration.Slot0.kP = 0.5;
        configuration.Slot0.kI = 0;
        configuration.Slot0.kD = 0.03;
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

        Phoenix6Util.applyAndCheckConfiguration(ampBar, configuration);
    }

    double degreesToMotorRotations(double degrees) {
        return degrees * MOTOR_ROTATIONS_PER_DEGREE;
    }

    @Override
    public void setSetpointInDegrees(double setpointInDegrees) {
        ampBar.setControl(ampBarRequest.withPosition(degreesToMotorRotations(setpointInDegrees)));
    }

    @Override
    public void setHomingPosition(double position) {
        ampBar.setPosition(degreesToMotorRotations(position));
    }

    @Override
    public void updateInputs(AmpBarIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                        ampBar.getMotorVoltage(),
                        ampBar.getSupplyCurrent(),
                        ampBar.getDeviceTemp(),
                        ampBar.getPosition(),
                        ampBar.getVelocity())
                .isOK();
        inputs.ampBarVoltage = ampBar.getMotorVoltage().getValueAsDouble();
        inputs.ampBarCurrent = ampBar.getSupplyCurrent().getValueAsDouble();
        inputs.ampBarTemperature = ampBar.getDeviceTemp().getValueAsDouble();
        inputs.ampBarPositionDegrees = ampBar.getPosition().getValueAsDouble() * DEGREES_PER_ROTATION;
        inputs.ampBarVelocityDegrees = ampBar.getVelocity().getValueAsDouble() * DEGREES_PER_ROTATION;
    }

    @Override
    public void setPercentage(double percentage) {
        ampBar.set(percentage);
    }
}
