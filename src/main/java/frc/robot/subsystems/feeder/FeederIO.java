package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    default void setTargetVoltages(double teacupVoltage, double slurpVoltage) {}

    @AutoLog
    class FeederIOInputs {
        public boolean teacupMotorConnected = true;
        public boolean feederMotorLeftConnected = true;
        public boolean feederMotorRightConnected = true;
        public double teacupVoltage = 0.0;
        public double slurpVoltage = 0.0;
        public double slurpFollowerVoltage = 0.0;
        public double teacupCurrent = 0.0;
        public double slurpCurrent = 0.0;
        public double slurpFollowerCurrent = 0.0;
        public double teacupTemperature = 0.0;
        public double slurpTemperature = 0.0;
        public double slurpFollowerTemperature = 0.0;
        public boolean beamBreakTripped = false;
    }

    default void updateInputs(FeederIOInputs inputs) {}
}
