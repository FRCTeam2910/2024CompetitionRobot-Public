package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    default void setSetpoint(double setpointInDegrees) {}

    default void setHomingPosition(double position) {}

    default void setPercentage(double percentage) {}

    default void disableSoftLimits() {}

    default void updateInputs(ClimberIOInputs inputs) {}

    @AutoLog
    class ClimberIOInputs {
        public boolean connected = true;
        public double climberVoltage = 0.0;
        public double climberCurrent = 0.0;
        public double climberTemperature = 0.0;
        public double climberPositionMotorRotations = 0.0;
        public double climberVelocityMotorRotations = 0.0;
    }
}
