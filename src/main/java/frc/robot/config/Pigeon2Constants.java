package frc.robot.config;

import frc.robot.util.drivers.CanDeviceId;

@SuppressWarnings({"InconsistentCapitalization", "VariableNameSameAsType"})
public class Pigeon2Constants {
    /** CAN ID of and Name of CANivore the Pigeon2 is on */
    public CanDeviceId CanDeviceId = new CanDeviceId(-1);

    /**  The mounting calibration yaw-component in degrees. */
    public double MountPose = 0.0;

    /** The gyro scalar component for the Z axis */
    public double GyroScalarZ = 0.0;

    public Pigeon2Constants withCanDeviceId(CanDeviceId id) {
        CanDeviceId = id;
        return this;
    }

    public Pigeon2Constants withMountPoseYaw(double yaw) {
        this.MountPose = yaw;
        return this;
    }

    public Pigeon2Constants withGyroScalarZ(double scalar) {
        this.GyroScalarZ = scalar;
        return this;
    }
}
