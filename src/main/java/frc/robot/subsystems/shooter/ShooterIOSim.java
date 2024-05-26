package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class ShooterIOSim implements ShooterIO {

    private final FlywheelSim leftRollers =
            new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.025); // numbers pulled from nowhere
    private final FlywheelSim rightRollers = new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.025);

    private final PIDController leftRollerFeedback = new PIDController(1.5, 0, 0.5);
    private final PIDController rightRollerFeedback = new PIDController(1.5, 0, 0.5);

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;

    private static final double ROLLER_RADIUS_METERS = 0.05;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftRollers.update(Robot.defaultPeriodSecs);
        rightRollers.update(Robot.defaultPeriodSecs);

        inputs.leftVoltage = leftAppliedVolts;
        inputs.leftRPM = leftRollers.getAngularVelocityRPM();
        inputs.rightVoltage = rightAppliedVolts;
        inputs.rightRPM = rightRollers.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double leftAppliedVolts, double rightAppliedVolts) {
        this.leftAppliedVolts = leftAppliedVolts;
        leftRollers.setInputVoltage(leftAppliedVolts);
        this.rightAppliedVolts = rightAppliedVolts;
        rightRollers.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void setRPM(double leftTargetRPM, double rightTargetRPM) {
        // rotations per minute times circumference in meters gives meters traveled per minute and then divide by 60 for
        // meters per second
        double leftVelocityFromRPM = leftTargetRPM * ROLLER_RADIUS_METERS * 2 * Math.PI / 60;
        leftAppliedVolts = leftRollerFeedback.calculate(leftRollers.getAngularVelocityRPM(), leftVelocityFromRPM);
        leftAppliedVolts = MathUtil.clamp(leftAppliedVolts, -12, 12);
        leftRollers.setInputVoltage(leftAppliedVolts);

        double rightVelocityFromRPM = rightTargetRPM * ROLLER_RADIUS_METERS * 2 * Math.PI / 60;
        rightAppliedVolts = rightRollerFeedback.calculate(rightRollers.getAngularVelocityRPM(), rightVelocityFromRPM);
        rightAppliedVolts = MathUtil.clamp(rightAppliedVolts, -12, 12);
        rightRollers.setInputVoltage(rightAppliedVolts);
    }
}
