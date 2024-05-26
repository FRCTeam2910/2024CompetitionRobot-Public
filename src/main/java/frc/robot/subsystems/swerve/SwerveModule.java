package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.config.SwerveModuleConfiguration;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    public final String name;
    private Rotation2d steerRelativeOffset = null; // Relative + Offset = Absolute
    private final SwerveModuleConfiguration swerveModuleConfig;

    // Alerts
    private final Alert driveMotorDisconnected;
    private final Alert steerMotorDisconnected;

    /**
     * Initializes a new instance of the SwerveModule class.
     *
     * @param io                 An instance of {@link SwerveModuleIO} class.
     * @param name               The identification name of the swerve module.
     * @param swerveModuleConfig The configuration for the swerve module.
     */
    public SwerveModule(SwerveModuleIO io, String name, SwerveModuleConfiguration swerveModuleConfig) {
        this.io = io;
        this.name = name;
        driveMotorDisconnected = new Alert(name + " drive motor disconnected!", Alert.AlertType.WARNING);
        steerMotorDisconnected = new Alert(name + " steer motor disconnected!", Alert.AlertType.WARNING);

        this.swerveModuleConfig = swerveModuleConfig;
    }

    /**
     * Reads the inputs without running the rest of the periodic logic. This is useful
     * since these updates need to be properly thread-locked.
     */
    public void updateInputs() {
        io.updateInputs(inputs);

        // Display alerts
        driveMotorDisconnected.set(!inputs.driveMotorConnected);
        steerMotorDisconnected.set(!inputs.steerMotorConnected);
    }

    public void periodic() {
        Logger.processInputs("Drive/Module" + name, inputs);
        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        if (steerRelativeOffset == null && inputs.steerAbsolutePosition.getRadians() != 0.0) {
            steerRelativeOffset = inputs.steerAbsolutePosition.minus(inputs.steerPosition);
        }
    }

    /**
     * Gets the current swerve module position (distance and angle)
     *
     * @return The current swerve module position (distance  and angle) as a {@link SwerveModulePosition} object.
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Gets the current swerve module state (speed and angle)
     *
     * @return the current swerve module state (speed and angle) as a {@link SwerveModuleState} object.
     */
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Sets the state for the module.
     *
     * @param optimizedState Desired {@link SwerveModuleState} for the module.
     * @return The optimized state.
     */
    public SwerveModuleState setTargetState(SwerveModuleState optimizedState, boolean steerMotionMagicEnabled) {
        // Optimize (i.e. minimize) the change in heading for the swerve module requires
        // reversing the direction the wheel spins. Odometry will still be accurate as both
        // steer angle and wheel speeds will have their signs "flipped."
        optimizedState = SwerveModuleState.optimize(optimizedState, getAngle());
        io.setModuleState(optimizedState, steerMotionMagicEnabled);
        return optimizedState;
    }

    /**
     * Gets the current drive position of the module in meters.
     *
     * @return the current drive position of the module in meters.
     */
    public double getPositionMeters() {
        return inputs.drivePositionRad * swerveModuleConfig.WheelRadiusInMeters;
    }

    /**
     * Gets the current drive velocity of the module in meters per second.
     *
     * @return The current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * swerveModuleConfig.WheelRadiusInMeters;
    }

    /**
     * Gets the current drive position of the module in meters.
     *
     * @return The current turn angle of the module.
     */
    public Rotation2d getAngle() {
        if (steerRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.steerPosition.plus(steerRelativeOffset);
        }
    }

    /**
     * Gets the current timestamp of the odometry reading.
     *
     * @return The current timestamp of the odometry reading.
     */
    public double getOdometryTimestamp() {
        return inputs.timestamp;
    }
}
