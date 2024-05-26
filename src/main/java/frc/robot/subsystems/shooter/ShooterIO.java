package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    default void setVoltage(double leftAppliedVolts, double rightAppliedVolts) {}

    default void setRPM(double leftTargetRPM, double rightTargetRPM) {}

    default void updateInputs(ShooterIOInputs inputs) {}

    @AutoLog
    class ShooterIOInputs {
        public boolean rightConnected = true;
        public boolean leftConnected = true;
        public double leftVoltage;
        public double rightVoltage;
        public double leftCurrent;
        public double rightCurrent;
        public double leftTemperature;
        public double rightTemperature;
        public double leftRPM;
        public double rightRPM;
    }
}
