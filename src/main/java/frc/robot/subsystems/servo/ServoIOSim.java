package frc.robot.subsystems.servo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.config.ServoMotorConfiguration;
import frc.robot.subsystems.servo.ServoSubsystem.ControlState;

public class ServoIOSim implements ServoIO {
    private final ProfiledPIDController feedback;
    private final ArmFeedforward feedforward;
    protected final DCMotorSim master;
    protected double forwardSoftLimitRotations;
    protected double reverseSoftLimitRotations;
    protected double appliedVolts = 0.0;

    public ServoIOSim(ServoMotorConfiguration config) {

        master = new DCMotorSim(DCMotor.getKrakenX60(1), config.SensorToMechanismRatio, 0.01);
        feedback = new ProfiledPIDController(10.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3 * Math.PI, 15), 0.01);
        feedforward = new ArmFeedforward(0.05, 0.0, 1.0, 0.0);

        forwardSoftLimitRotations = (((config.MaxUnitsLimit - config.HomePosition) * config.RotationsPerUnitDistance)
                - config.SoftLimitDeadband);
        reverseSoftLimitRotations = (((config.MinUnitsLimit - config.HomePosition) * config.RotationsPerUnitDistance)
                + config.SoftLimitDeadband);
    }

    @Override
    public void updateInputs(ServoIOInputs inputs) {
        // Update every period - use a set amount of seconds.
        master.update(Robot.defaultPeriodSecs);

        inputs.timestamp = Timer.getFPGATimestamp();
        inputs.positionRotations = Units.radiansToRotations(master.getAngularPositionRad());
        inputs.velocityRotationsSecond = Units.radiansToRotations(master.getAngularVelocityRadPerSec());
        inputs.outputVoltage = appliedVolts;
        inputs.masterCurrent = Math.abs(master.getCurrentDrawAmps());
    }

    @Override
    public void writeOutputs(ControlState controlState, double demand, double feedforward) {
        // Calculate demand to voltage data

        if (controlState != ServoSubsystem.ControlState.OPEN_LOOP) {
            // demand is a constrained position in units of mechanism rotations
            feedback.setGoal(Units.rotationsToRadians(demand));
            for (int i = 0; i < 20; i++) {
                appliedVolts = feedback.calculate(master.getAngularPositionRad())
                        + this.feedforward.calculate(feedback.getSetpoint().position, feedback.getSetpoint().velocity);
                appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
                master.setInputVoltage(appliedVolts);
            }
        } else {
            // demand is in volts
            appliedVolts = demand;
            appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
            master.setInputVoltage(appliedVolts);
        }
    }
}
