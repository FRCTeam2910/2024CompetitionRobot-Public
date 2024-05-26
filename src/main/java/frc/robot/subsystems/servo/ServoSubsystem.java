// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.servo;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ServoMotorConfiguration;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 */
public abstract class ServoSubsystem extends SubsystemBase {

    public enum ControlState {
        OPEN_LOOP,
        MOTION_MAGIC,
        POSITION_PID,
        MOTION_PROFILING
    }

    protected ControlState controlState = ControlState.OPEN_LOOP;

    protected final ServoIOInputsAutoLogged inputs = new ServoIOInputsAutoLogged();
    protected final ServoIO servo;
    protected ServoIO.ServoIOOutputs controlOutputs = new ServoIO.ServoIOOutputs();
    protected ServoMotorConfiguration config;
    private double forwardSoftLimitRotations;
    private double reverseSoftLimitRotations;

    protected double forwardMaxUnitsLimit = 0.0;
    protected double reverseMinUnitsLimit = 0.0;

    private final Alert disconnected;

    /**
     * Creates a new ServoSubsystem.
     *
     * @param servo The ServoIO object that this subsystem will interface with.
     * @param config The configuration for the servo motor.
     */
    protected ServoSubsystem(ServoIO servo, ServoMotorConfiguration config) {
        this.servo = servo;
        this.config = config;

        forwardMaxUnitsLimit = config.MaxUnitsLimit;
        reverseMinUnitsLimit = config.MinUnitsLimit;
        forwardSoftLimitRotations = (((forwardMaxUnitsLimit - config.HomePosition) * config.RotationsPerUnitDistance)
                - config.SoftLimitDeadband);
        reverseSoftLimitRotations = (((reverseMinUnitsLimit - config.HomePosition) * config.RotationsPerUnitDistance)
                + config.SoftLimitDeadband);
        disconnected = new Alert(config.Name + " motor disconnected!", Alert.AlertType.WARNING);
    }

    @Override
    public void periodic() {
        // read inputs
        servo.updateInputs(inputs);
        // log inputs
        Logger.processInputs(config.Name + "/Inputs", inputs);

        if (inputs.resetOccured) {
            servo.handleReset();
            resetIfAtHome();
        }

        // write outputs
        servo.writeOutputs(controlState, controlOutputs.demand, controlOutputs.feedforward);

        // Alerts
        disconnected.set(!inputs.connected);
    }

    public void setOpenLoop(double percentage) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        controlOutputs.demand = percentage;
    }

    public double unitsToRotations(double units) {
        return units * config.RotationsPerUnitDistance;
    }

    public double rotationsToUnits(double rotations) {
        return rotations / config.RotationsPerUnitDistance;
    }

    public double rotationsToHomedUnits(double rotations) {
        double val = rotationsToUnits(rotations);
        return val + config.HomePosition;
    }

    public double homeAwareUnitsToRotations(double units) {
        return unitsToRotations(units - config.HomePosition);
    }

    public double constrainRotations(double rotations) {
        return MathUtil.clamp(rotations, reverseSoftLimitRotations, forwardSoftLimitRotations);
    }

    // In "Units"
    public double getPosition() {
        return rotationsToHomedUnits(inputs.positionRotations);
    }

    public double getPositionRotations() {
        return inputs.positionRotations;
    }

    public double getVelocity() {
        return rotationsToUnits(inputs.velocityRotationsSecond);
    }

    public synchronized boolean hasFinishedTrajectory() {
        return MathUtil.isNear(inputs.activeTrajectoryPosition, getSetpoint(), Math.max(1.0, config.Deadband));
    }

    public boolean atHomingLocation() {
        return false;
    }

    public void resetIfAtHome() {
        if (atHomingLocation()) {
            zeroSensors();
        }
    }

    public void zeroSensors() {
        servo.zeroSensors();
    }

    public void setSensorPosition(double units) {
        servo.setSensorPosition(units);
    }

    protected void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
        servo.changeTalonConfig(configChanger);
    }

    public void enableSoftLimits(boolean enable) {
        changeTalonConfig(conf -> {
            conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
            conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
            return conf;
        });
    }

    public void setNeutralMode(NeutralModeValue mode) {
        changeTalonConfig(conf -> {
            conf.MotorOutput.NeutralMode = mode;
            return conf;
        });
    }

    public void setStatorCurrentLimit(double currentLimit, boolean enable, boolean unchecked) {
        servo.setStatorCurrentLimit(currentLimit, enable, unchecked);
    }

    public synchronized void setSupplyCurrentLimit(double value, boolean enable, boolean unchecked) {
        servo.setSupplyCurrentLimit(value, enable, unchecked);
    }

    public synchronized void setMotionMagicConfigs(double accel, double jerk, boolean unchecked) {
        servo.setMotionMagicConfigs(accel, jerk, unchecked);
    }

    public void rewriteDeviceConfiguration() {
        servo.writeConfigs();
    }

    public double getSetpoint() {
        return (controlState == ControlState.MOTION_MAGIC
                        || controlState == ControlState.POSITION_PID
                        || controlState == ControlState.MOTION_PROFILING)
                ? rotationsToHomedUnits(controlOutputs.demand)
                : Double.NaN;
    }

    public double getSetpointHomed() {
        return (controlState == ControlState.MOTION_MAGIC
                        || controlState == ControlState.POSITION_PID
                        || controlState == ControlState.MOTION_PROFILING)
                ? rotationsToHomedUnits(controlOutputs.demand)
                : Double.NaN;
    }

    public synchronized void setSetpointPositionPID(double units, double feedforward) {
        controlOutputs.demand = constrainRotations(homeAwareUnitsToRotations(units));
        controlOutputs.feedforward = feedforward;
        // FEEDFORWARD DELETED BECAUSE WE WANT TO SET IT DIRECTLY - NO CONVERWSION
        //        controlOutputs.feedforward =
        //                feedforward_ticks_per_100ms * (config.slot1MotionMagickV + config.slot1MotionMagickD / 100.0)
        // / 1023.0;
        if (controlState != ControlState.POSITION_PID) {
            controlState = ControlState.POSITION_PID;
        }
    }

    public synchronized void setSetpointPositionPID(double units) {
        setSetpointPositionPID(units, 0.0);
    }

    public void setSetpointMotionMagic(double units, double feedforward) {
        controlOutputs.demand = constrainRotations(homeAwareUnitsToRotations(units));
        controlOutputs.feedforward = config.ArbitraryFeedforward;
        if (controlState != ControlState.MOTION_MAGIC) {
            controlState = ControlState.MOTION_MAGIC;
        }
    }

    public void setSetpointMotionMagic(double units) {
        setSetpointMotionMagic(units, 0.0);
    }

    //    public void setSetpointPositionPID(double units, double feedforward_v) {
    //        controlOutputs.demand = servo.constrainRotations(servo.homeAwareUnitsToRotations(units));
    //        double feedforward_ticks_per_100ms = servo.unitsToRotations(feedforward_v);
    //        controlOutputs.feedforward = feedforward_ticks_per_100ms * (config.kF + config.kD / 100.0) / 1023.0;
    //        if (controlState != ControlState.POSITION_PID) {
    //            controlState = ControlState.POSITION_PID;
    //        }
    //    }

    //    public void setSetpointPositionPID(double units) {
    //        setSetpointPositionPID(units, 0.0);
    //    }

    //    public synchronized double getActiveTrajectoryPosition() {
    //        return servo.rotationsToHomedUnits((inputs.active_trajectory_position));
    //    }

    public void stop() {
        setOpenLoop(0.0);
        servo.stop();
    }
}
