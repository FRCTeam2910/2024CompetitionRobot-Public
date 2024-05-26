package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    default void updateInputs(IntakeIOInputs inputs) {}

    @AutoLog
    class IntakeIOInputs {
        public boolean topMotorConnected = true;
        public boolean bottomMotorConnected = true;
        public double topVoltage = 0.0;
        public double bottomVoltage = 0.0;
        public double topCurrent = 0.0;
        public double bottomCurrent = 0.0;
        public double topTemperature = 0.0;
        public double bottomTemperature = 0.0;
        public double topVelocityRPS = 0.0;
        public double bottomVelocityRPS = 0.0;
    }

    default void setTopMotorVoltage(double voltage) {}

    default void setBottomMotorVoltage(double voltage) {}
}
