package frc.robot.subsystems.swerve;

import frc.robot.util.drivers.CanDeviceId;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    /**
     * Reads sensor data (hardware or simulation) and updates the inputs parameter.
     *
     * @param inputs Contains the defaults for the input values listed above.
     */
    default void updateInputs(GyroIOInputs inputs) {}

    /**
     * Holds data that can be read a gyroscope IO implementation.
     */
    @AutoLog
    class GyroIOInputs {
        public boolean connected = true;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double roll = 0.0;
        public double angularVelocity = 0.0;
        public double angularVelocityDegrees = 0.0;
        public double accelerationX = 0.0;
        public double accelerationY = 0.0;
    }

    /**
     * Gets the CANDeviceId.
     *
     * @return the CANDeviceId
     */
    default CanDeviceId getCanDeviceId() {
        return new CanDeviceId(-1);
    }
}
