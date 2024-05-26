package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class IntakeIOSim implements IntakeIO {
    private DCMotorSim bottomMotor;
    private DCMotorSim topMotor;
    private double bottomVoltage;
    private double topVoltage;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        bottomMotor.update(Robot.defaultPeriodSecs);
        topMotor.update(Robot.defaultPeriodSecs);
        inputs.bottomCurrent = bottomMotor.getCurrentDrawAmps();
        inputs.bottomTemperature = 0;
        inputs.bottomVoltage = bottomVoltage;
        inputs.topCurrent = topMotor.getCurrentDrawAmps();
        inputs.topTemperature = 0;
        inputs.topVoltage = topVoltage;
    }

    public IntakeIOSim() {
        bottomMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.1);
        topMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.1);
    }

    @Override
    public void setBottomMotorVoltage(double voltage) {
        bottomVoltage = voltage;
        bottomMotor.setInputVoltage(voltage);
    }

    @Override
    public void setTopMotorVoltage(double voltage) {
        topVoltage = voltage;
        topMotor.setInputVoltage(voltage);
    }
}
