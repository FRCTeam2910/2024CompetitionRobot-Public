package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    default void setSetpointInDegrees(double setpointInDegrees) {}

    default void setHomingPosition(double position) {}

    default void disableSoftLimits() {}

    default void updateInputs(PivotIOInputs inputs) {}

    default void setPercentage(double percentage) {}

    @AutoLog
    class PivotIOInputs {
        public boolean connected = true;
        public double pivotVoltage = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotTemperature = 0.0;
        public double pivotPositionDegrees = 0.0;
        public double pivotVelocityDegrees = 0.0;
    }
}
