package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface CANCoderIO {
    default void updateInputs(CANCoderIOInputs inputs) {}

    @AutoLog
    class CANCoderIOInputs {
        public double positionRotations = 0.0;
        public double positionDegrees = 0.0;
    }
}
