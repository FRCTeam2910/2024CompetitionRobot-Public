package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.config.PortConfiguration;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonFXFactory;
import org.littletonrobotics.junction.Logger;

public class ClimberIOPhoenix6 implements ClimberIO {
    public final TalonFX climber;
    private final TalonFXConfiguration configuration = new TalonFXConfiguration();
    private static final double FORWARD_SOFT_LIMIT_MOTOR_ROTATIONS = 107.0;
    private static final double REVERSE_SOFT_LIMIT_MOTOR_ROTATIONS = -1.0;
    private static final int MOTION_MAGIC_SLOT = 0;

    private final MotionMagicDutyCycle motionMagicDutyCycleRequest =
            new MotionMagicDutyCycle(0.0, false, 0.0, MOTION_MAGIC_SLOT, false, false, false);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0, false, false, false, false);

    public ClimberIOPhoenix6(PortConfiguration ports) {
        climber = TalonFXFactory.createDefaultTalon(ports.climberID);

        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configuration.CurrentLimits.SupplyCurrentLimit = 10.0;
        configuration.CurrentLimits.StatorCurrentLimit = 150.0;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimitEnable = false;

        configuration.CurrentLimits.SupplyCurrentThreshold = 30.0;
        configuration.CurrentLimits.SupplyTimeThreshold = 1.275;

        configuration.Slot0.kP = 1.0;
        configuration.Slot0.kI = 0;
        configuration.Slot0.kD = 0;
        configuration.Slot0.kV = 0;
        configuration.Slot0.kA = 0;
        configuration.Slot0.kS = 0;

        configuration.MotionMagic.MotionMagicCruiseVelocity = 50.0;
        configuration.MotionMagic.MotionMagicAcceleration = 100.0;
        configuration.MotionMagic.MotionMagicJerk = 0.0;

        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT_MOTOR_ROTATIONS;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT_MOTOR_ROTATIONS;

        // Start with soft limits disabled.
        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;

        Phoenix6Util.applyAndCheckConfiguration(climber, configuration);
    }

    @Override
    public void setSetpoint(double setpointInMotorRotations) {
        if (MathUtil.isNear(setpointInMotorRotations, climber.getPosition().getValueAsDouble(), 0.5)) {
            climber.set(0.0);
        } else {
            climber.setControl(motionMagicDutyCycleRequest.withPosition(setpointInMotorRotations));
        }
    }

    @Override
    public void setHomingPosition(double position) {
        climber.setPosition(-0.5);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                        climber.getMotorVoltage(),
                        climber.getSupplyCurrent(),
                        climber.getDeviceTemp(),
                        climber.getPosition(),
                        climber.getVelocity())
                .isOK();
        inputs.climberVoltage = climber.getMotorVoltage().getValueAsDouble();
        inputs.climberCurrent = climber.getSupplyCurrent().getValueAsDouble();
        inputs.climberTemperature = climber.getDeviceTemp().getValueAsDouble();
        inputs.climberPositionMotorRotations = climber.getPosition().getValueAsDouble();
        inputs.climberVelocityMotorRotations = climber.getVelocity().getValueAsDouble();

        Logger.recordOutput("ClimberPercentOut", climber.getDutyCycle().getValueAsDouble());
        Logger.recordOutput("ClimberSupplyVoltage", climber.getSupplyVoltage().getValueAsDouble());
    }

    @Override
    public void setPercentage(double percentage) {
        climber.setControl(dutyCycleOut.withOutput(percentage));
    }
}
