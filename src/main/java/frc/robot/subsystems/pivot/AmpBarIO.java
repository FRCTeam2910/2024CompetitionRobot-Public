package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface AmpBarIO {
    default void setSetpointInDegrees(double setpointInDegrees) {}

    default void setHomingPosition(double position) {}

    default void disableSoftLimits() {}

    default void updateInputs(AmpBarIOInputs inputs) {}

    default void setPercentage(double percentage) {}

    @AutoLog
    class AmpBarIOInputs {
        public boolean connected = true;
        public double ampBarVoltage = 0.0;
        public double ampBarCurrent = 0.0;
        public double ampBarTemperature = 0.0;
        public double ampBarPositionDegrees = 0.0;
        public double ampBarVelocityDegrees = 0.0;
    }
}
