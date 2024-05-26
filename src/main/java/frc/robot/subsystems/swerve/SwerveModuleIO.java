package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.drivers.CanDeviceId;
import org.littletonrobotics.junction.AutoLog;

/**
 * Represents a hardware abstraction layer for a Swerve Module that
 * supports logging with AdvantageKit.
 */
public interface SwerveModuleIO {
    /**
     * Update the {@link SwerveModuleIOInputs} instance with latest sensor inputs.
     *
     * @param swerveModuleIOInputs A {@link SwerveModuleIOInputs} instance to update.
     */
    default void updateInputs(SwerveModuleIOInputs swerveModuleIOInputs) {}

    /**
     * Sets the target state for the module.
     *
     * @param state The target state.
     */
    default void setModuleState(SwerveModuleState state, boolean steerMotionMagicEnabled) {}

    /**
     * Gets the CanDeviceId for the module.
     *
     * @return The {@link CanDeviceId}.
     */
    default CanDeviceId getCanDeviceId() {
        return new CanDeviceId(-1);
    }

    /**
     * IO values for the swerve module.
     */
    @AutoLog
    class SwerveModuleIOInputs {
        public boolean driveMotorConnected = true;
        public boolean steerMotorConnected = true;

        // Drive Inputs
        public double timestamp = 0.0;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveTemperature = 0.0;

        // Steering Inputs
        /**
         * Absolute position. Angle stays the same at all times (as opposed to the
         * motor encoders which update the angles based off of the starting angle).
         */
        public Rotation2d steerAbsolutePosition = new Rotation2d();

        public double steerAbsolutePositionRadians = 0.0;
        public Rotation2d steerPosition = new Rotation2d();
        public double steerPositionRad = 0.0;
        public double steerPositionDeg = 0.0;
        public double steerVelocityRadPerSec = 0.0;
        public double steerCurrentDrawAmps = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerTemperature = 0.0;

        // Drive Outputs
        public double targetDriveVelocityMetersPerSec = 0.0;

        // Steer Outputs
        public double targetSteerPositionRad = 0.0;
    }
}
