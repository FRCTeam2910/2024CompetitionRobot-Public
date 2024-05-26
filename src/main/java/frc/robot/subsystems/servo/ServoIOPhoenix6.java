// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.servo;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.ServoMotorConfiguration;
import frc.robot.subsystems.servo.ServoSubsystem.ControlState;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonFXFactory;

import static com.ctre.phoenix6.BaseStatusSignal.setUpdateFrequencyForAll;

@SuppressWarnings("UnusedMethod")
public class ServoIOPhoenix6 implements ServoIO {
    private static final int MOTION_MAGIC_SLOT = 1;
    private static final int POSITION_SLOT = 0;

    private final ServoMotorConfiguration config;

    protected final TalonFX leader;
    protected final TalonFX[] followers;

    private TalonFXConfiguration leaderConfig;
    protected final TalonFXConfiguration[] followerConfigs;

    private final StatusSignal<Double> positionSignal;
    private final StatusSignal<Double> velocitySignal;
    private final StatusSignal<Double> closedLoopError;
    private final StatusSignal<Double> statorCurrentSignal;
    private final StatusSignal<Double> outputVoltageSignal;
    private final StatusSignal<Double> outputPercentageSignal;
    private final StatusSignal<Double> closedLoopOutputSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;
    private final StatusSignal<Double> closedLoopReferenceSlopeSignal;

    protected StatusSignal<Integer> stickyFault;
    protected double forwardSoftLimitRotations;
    protected double reverseSoftLimitRotations;

    boolean hasBeenZeroed = false;

    public ServoIOPhoenix6(ServoMotorConfiguration config) {
        this.config = config;
        leader = TalonFXFactory.createDefaultTalon(config.MasterConstants.canDeviceId);
        followers = new TalonFX[config.FollowerConstants.length];
        followerConfigs = new TalonFXConfiguration[config.FollowerConstants.length];

        leaderConfig = new TalonFXConfiguration();
        leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        forwardSoftLimitRotations = (((config.MaxUnitsLimit - config.HomePosition) * config.RotationsPerUnitDistance)
                - config.SoftLimitDeadband);
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimitRotations;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        reverseSoftLimitRotations = (((config.MinUnitsLimit - config.HomePosition) * config.RotationsPerUnitDistance)
                + config.SoftLimitDeadband);
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimitRotations;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        leaderConfig.Slot0.kP = config.slot0PositionPIDkP;
        leaderConfig.Slot0.kI = config.slot0PositionPIDkI;
        leaderConfig.Slot0.kD = config.slot0PositionPIDkD;
        leaderConfig.Slot0.kV = config.slot0PositionPIDkV;
        leaderConfig.Slot0.kA = config.slot0PositionPIDkA;
        leaderConfig.Slot0.kS = config.slot0PositionPIDkS;

        leaderConfig.Slot1.kP = config.slot1MotionMagickP;
        leaderConfig.Slot1.kI = config.slot1MotionMagickI;
        leaderConfig.Slot1.kD = config.slot1MotionMagickD;
        leaderConfig.Slot1.kV = config.slot1MotionMagickV;
        leaderConfig.Slot1.kA = config.slot1MotionMagickA;
        leaderConfig.Slot1.kS = config.slot1MotionMagickS;

        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = config.CruiseVelocity;
        leaderConfig.MotionMagic.MotionMagicAcceleration = config.Acceleration;
        leaderConfig.MotionMagic.MotionMagicJerk = config.Jerk;

        leaderConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.RampRate;
        leaderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.RampRate;
        leaderConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = config.RampRate;

        leaderConfig.CurrentLimits.SupplyCurrentLimit = config.SupplyCurrentLimit;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = config.EnableSupplyCurrentLimit;
        leaderConfig.CurrentLimits.StatorCurrentLimit = config.StatorCurrentLimit;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = config.EnableStatorCurrentLimit;

        leaderConfig.MotorOutput.Inverted = (config.MasterConstants.counterClockwisePositive
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive);

        // TODO - because the demand already applies gear rations, we don't need to do it here.  Consider changing this.
        leaderConfig.Feedback.SensorToMechanismRatio = (config.MasterConstants.invertMotorOutput ? -1.0 : 1.0);
        leaderConfig.MotorOutput.NeutralMode = config.NeutralMode;

        for (int i = 0; i < followers.length; ++i) {
            TalonFX follower = TalonFXFactory.createPermanentFollowerTalon(
                    config.FollowerConstants[i].canDeviceId,
                    config.MasterConstants.canDeviceId,
                    config.FollowerConstants[i].invertMotorOutput);

            followers[i] = follower;
            TalonFXConfiguration followerConfig = followerConfigs[i];
            Phoenix6Util.checkErrorAndRetry(() -> follower.getConfigurator().refresh(followerConfig));
            followerConfig.MotorOutput.NeutralMode = config.NeutralMode;
            Phoenix6Util.applyAndCheckConfiguration(follower, followerConfig);
            followerConfigs[i] = followerConfig;
        }

        // Which one do we want? Test both and see
        Phoenix6Util.applyAndCheckConfiguration(leader, leaderConfig);

        positionSignal = leader.getRotorPosition().clone();
        velocitySignal = leader.getRotorVelocity().clone();
        closedLoopError = leader.getClosedLoopError().clone();
        statorCurrentSignal = leader.getStatorCurrent().clone();
        outputVoltageSignal = leader.getSupplyVoltage().clone();
        outputPercentageSignal = leader.getDutyCycle().clone();
        closedLoopOutputSignal = leader.getClosedLoopOutput().clone();
        closedLoopReferenceSignal = leader.getClosedLoopReference().clone();
        closedLoopReferenceSlopeSignal = leader.getClosedLoopReferenceSlope().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                velocitySignal,
                closedLoopError,
                statorCurrentSignal,
                outputVoltageSignal,
                outputPercentageSignal,
                closedLoopReferenceSignal,
                closedLoopOutputSignal,
                closedLoopReferenceSlopeSignal);
    }

    @Override
    public void updateInputs(ServoIOInputs inputs) {
        inputs.timestamp = Timer.getFPGATimestamp();

        stickyFault = leader.getStickyFaultField();

        inputs.connected = BaseStatusSignal.refreshAll(
                        velocitySignal,
                        positionSignal,
                        statorCurrentSignal,
                        outputVoltageSignal,
                        outputPercentageSignal,
                        closedLoopError,
                        closedLoopOutputSignal,
                        closedLoopReferenceSlopeSignal)
                .isOK();

        double masterRotations = BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal);

        if (leader.getControlMode().getValue() == ControlModeValue.PositionDutyCycle) {
            inputs.errorRotations = closedLoopError.asSupplier().get();
        } else {
            inputs.errorRotations = 0;
        }
        inputs.masterCurrent = statorCurrentSignal.asSupplier().get();
        inputs.outputVoltage = outputVoltageSignal.asSupplier().get();
        inputs.outputPercent = outputPercentageSignal.asSupplier().get();
        inputs.positionRotations = masterRotations;
        inputs.positionUnits = rotationsToHomedUnits(inputs.positionRotations);
        inputs.velocityRotationsSecond = velocitySignal.asSupplier().get();
        inputs.activeTrajectoryPosition = closedLoopReferenceSignal.asSupplier().get();

        final double newVelocity =
                BaseStatusSignal.getLatencyCompensatedValue(closedLoopReferenceSignal, closedLoopReferenceSlopeSignal);

        if (MathUtil.isNear(newVelocity, config.CruiseVelocity, Math.max(1, config.Deadband))
                || MathUtil.isNear(newVelocity, inputs.activeTrajectoryVelocity, Math.max(1, config.Deadband))) {
            // Mechanism is ~constant velocity.
            inputs.activeTrajectoryAcceleration = 0.0;
        } else {
            // Mechanism is accelerating.
            inputs.activeTrajectoryAcceleration =
                    Math.signum(newVelocity - inputs.activeTrajectoryVelocity) * config.Acceleration;
        }
        inputs.activeTrajectoryVelocity = newVelocity;
    }

    @Override
    public void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
        for (int i = 0; i < followerConfigs.length; ++i) {
            followerConfigs[i] = configChanger.apply(followerConfigs[i]);
        }
        leaderConfig = configChanger.apply(leaderConfig);
        writeConfigs();
    }

    @Override
    public void writeConfigs() {
        for (int i = 0; i < followers.length; ++i) {
            TalonFX slave = followers[i];
            TalonFXConfiguration slaveConfig = followerConfigs[i];
            Phoenix6Util.applyAndCheckConfiguration(slave, slaveConfig);
        }
        Phoenix6Util.applyAndCheckConfiguration(leader, leaderConfig);
    }

    @Override
    public void zeroSensors() {
        Phoenix6Util.checkErrorAndRetry(() -> leader.setPosition(0, config.kCANTimeout));
        hasBeenZeroed = true;
    }

    @Override
    public void setSensorPosition(double units) {
        Phoenix6Util.checkErrorAndRetry(() -> leader.setPosition(units, config.kCANTimeout));
        hasBeenZeroed = true;
    }

    @Override
    public synchronized void handleReset() {
        Phoenix6Util.checkErrorAndRetry(() -> setUpdateFrequencyForAll(
                200,
                positionSignal,
                velocitySignal,
                closedLoopError,
                statorCurrentSignal,
                outputVoltageSignal,
                outputPercentageSignal,
                closedLoopReferenceSignal,
                closedLoopOutputSignal,
                closedLoopReferenceSlopeSignal));
    }

    @Override
    public void setSupplyCurrentLimit(double value, boolean enable, boolean unchecked) {
        leaderConfig.CurrentLimits.SupplyCurrentLimit = value;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;
        if (unchecked) {
            leader.getConfigurator().apply(leaderConfig.CurrentLimits);
        } else {
            Phoenix6Util.applyAndCheckConfiguration(leader, leaderConfig);
        }
    }

    @Override
    public void setStatorCurrentLimit(double value, boolean enable, boolean unchecked) {
        leaderConfig.CurrentLimits.StatorCurrentLimit = value;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = enable;
        if (unchecked) {
            leader.getConfigurator().apply(leaderConfig.CurrentLimits);
        } else {
            Phoenix6Util.applyAndCheckConfiguration(leader, leaderConfig);
        }
    }

    @Override
    public void setMotionMagicConfigs(double accel, double jerk, boolean unchecked) {
        leaderConfig.MotionMagic.MotionMagicAcceleration = accel;
        leaderConfig.MotionMagic.MotionMagicJerk = jerk;
        if (unchecked) {
            leader.getConfigurator().apply(leaderConfig.MotionMagic);
        } else {
            Phoenix6Util.applyAndCheckConfiguration(leader, leaderConfig);
        }
    }

    @Override
    public void writeOutputs(ControlState controlState, double demand, double feedforward) {
        if (controlState == ControlState.MOTION_MAGIC) {
            leader.setControl(
                    new MotionMagicVoltage(demand).withFeedForward(feedforward).withSlot(MOTION_MAGIC_SLOT));
        } else if (controlState == ControlState.POSITION_PID)
        // || controlState == ControlState.MOTION_PROFILING)
        {
            leader.setControl(
                    new PositionDutyCycle(demand).withFeedForward(feedforward).withSlot(POSITION_SLOT));
        } else {
            leader.setControl(new DutyCycleOut(demand));
        }
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }

    @Override
    public TalonFXConfiguration getConfig() {
        return leaderConfig;
    }

    private double rotationsToUnits(double rotations) {
        return rotations / config.RotationsPerUnitDistance;
    }

    private double rotationsToHomedUnits(double rotations) {
        double val = rotationsToUnits(rotations);
        return val + config.HomePosition;
    }
}
