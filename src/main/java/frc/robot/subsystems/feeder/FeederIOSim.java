package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class FeederIOSim implements FeederIO {

    private final FlywheelSim teacupWheelsSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025); // All of these numbers will change
    private final FlywheelSim slurpRollersSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025); // All of these numbers will change
    private final FlywheelSim dongleSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025); // All of these numbers will change
    private final DIOSim beambrakeSim = new DIOSim(0);

    public FeederIOSim() {}

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        teacupWheelsSim.update(Robot.defaultPeriodSecs);
        slurpRollersSim.update(Robot.defaultPeriodSecs);
        dongleSim.update(Robot.defaultPeriodSecs);
        beambrakeSim.getValue();
    }

    @Override
    public void setTargetVoltages(double teacupVoltage, double slurpVoltage) {
        teacupWheelsSim.setInputVoltage(teacupVoltage);
        slurpRollersSim.setInputVoltage(slurpVoltage);
    }
}
