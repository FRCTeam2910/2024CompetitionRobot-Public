package frc.robot.config;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.drivers.CanDeviceId;

@SuppressWarnings({"InconsistentCapitalization", "EmptyBlockTag"})
public class SwerveModuleConfiguration {
    public enum SwerveModuleSteerFeedbackType {
        RemoteCANcoder,
        FusedCANcoder,
        SyncCANcoder,
    }

    /**
     * Name of the module
     */
    public String Name = "Swerve Module";

    /**
     * CanDeviceId for the drive motor
     */
    public CanDeviceId DriveCanDeviceId = new CanDeviceId(-1);

    /**
     * CanDeviceId for the steer motor
     */
    public CanDeviceId SteerCanDeviceId = new CanDeviceId(-1);

    /**
     * CanDeviceId for the CANcoder
     */
    public CanDeviceId CancoderCanDeviceId = new CanDeviceId(-1);

    /**
     * Offset of the CANcoder in rotations
     */
    public double CANcoderOffsetRotations = 0.0;

    /**
     * Gear ratio between drive motor and wheel
     */
    public double DriveMotorGearRatio = 0.0;

    /**
     * Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8
     */
    public double SteerMotorGearRatio = 0.0;

    /**
     * Coupled gear ratio between the CANcoder and the drive motor.
     * <p>
     * For a typical swerve module, the azimuth turn motor also drives the wheel a nontrivial
     * amount, which affects the accuracy of odometry and control. This ratio represents the
     * number of rotations of the drive motor caused by a rotation of the azimuth.
     */
    public double CouplingGearRatio = 0.0;

    /**
     * Wheel radius of the driving wheel in meters
     */
    public double WheelRadiusInMeters = 0.0;

    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     */
    Translation2d Location = new Translation2d(0, 0);

    /**
     * The steer motor closed loop gains
     */
    public Slot0Configs SteerMotorGains = new Slot0Configs();

    /**
     * The drive motor closed loop gains
     */
    public Slot0Configs DriveMotorGains = new Slot0Configs();

    /**
     * The maximum amount of supply current for Drive motor
     */
    public double DriveMotorSupplyCurrent = 50;

    /**
     * The maximum amount of stator current Drive motor
     */
    public double DriveMotorStatorCurrent = 200;

    /**
     * True if the drive motor supply current limit is enabled
     */
    public boolean EnableDriveMotorSupplyCurrentLimit = true;

    /**
     * True if the drive motor stator current limit is enabled
     */
    public boolean EnableDriveMotorStatorCurrentLimit = true;

    /**
     * True if the drive motor is reversed
     */
    public boolean DriveMotorInverted = false;

    /**
     * The maximum amount of supply current Steer motor
     */
    public double SteerMotorSupplyCurrent = 30;

    /**
     * The maximum amount of stator current Steer motor
     */
    public double SteerMotorStatorCurrent = 80;

    /**
     * True if the steer motor supply current limit is enabled
     */
    public boolean EnableSteerMotorSupplyCurrentLimit = true;

    /**
     * True if the steer motor stator current limit is enabled
     */
    public boolean EnableSteerMotorStatorCurrentLimit = true;

    /**
     * True if the steer motor is reversed from the CANcoder
     */
    public boolean SteerMotorInverted = false;

    /**
     * When using open-loop drive control, this specifies the speed the robot travels
     * at when driven with 12 volts in meters per second. This is used to approximate
     * the output for a desired velocity. If using closed loop control, this value is ignored
     */
    public double SpeedAt12VoltsMetersPerSecond = 0;

    /**
     * Whether to compensate for the cosine of steer motor angle error or not.
     * <p>When compensation is applied, the drive motor speed is scaled based \
     * on steer angle error.</p>
     * <p>When the error is 90°, the velocity setpoint should be 0. As the wheel turns
     * towards the setpoint, its velocity should increase. This is achieved by
     * taking the component of the velocity in the direction of the setpoint.
     * </p>
     */
    public boolean ApplyCosineCompensation = false;

    /**
     * Whether to compensate for the coupling between the drive and steer motors.
     * <p>
     * When compensation is applied, the drive motor position is adjusted based on the
     * steer motor position.
     * <p>
     * When the steer motor rotates, the drive motor rotates a nontrivial amount. This
     * affects the accuracy of odometry and control. This is achieved by
     * taking the component of the position in the direction of the setpoint.
     * </p>
     */
    public boolean ApplyCouplingCompensation = false;

    /**
     * Choose how the feedback sensors should be configured
     * <p>
     * If the robot does not support Pro, then this should remain as RemoteCANcoder.
     * Otherwise, users have the option to use either FusedCANcoder or SyncCANcoder depending
     * on if there is risk that the CANcoder can fail in a way to provide "good" data.
     */
    public SwerveModuleSteerFeedbackType FeedbackSource = SwerveModuleSteerFeedbackType.FusedCANcoder;

    /**
     * Sets the name of the module.
     *
     * @param name The name of the module.
     * @return this object
     */
    public SwerveModuleConfiguration withName(String name) {
        this.Name = name;
        return this;
    }

    /**
     * Sets the CANcoder offset in rotations.
     *
     * @param offset
     * @return this object
     */
    public SwerveModuleConfiguration withCANcoderOffsetRotations(double offset) {
        this.CANcoderOffsetRotations = offset;
        return this;
    }

    /**
     * Sets rhe gear ratio between the drive motor and the wheel.
     *
     * @param ratio
     * @return this object
     */
    public SwerveModuleConfiguration withDriveMotorGearRatio(double ratio) {
        this.DriveMotorGearRatio = ratio;
        return this;
    }

    /**
     * Sets the gear ratio between the steer motor and the CANcoder.
     * <p>
     * An example ratio for the SDS Mk4: 12.8
     *
     * @param ratio Gear ratio between the steer motor and the CANcoder
     * @return this object
     */
    public SwerveModuleConfiguration withSteerMotorGearRatio(double ratio) {
        this.SteerMotorGearRatio = ratio;
        return this;
    }

    /**
     * Sets the coupled gear ratio between the CANcoder and the drive motor.
     * <p>
     * For a typical swerve module, the steer motor also drives the wheel a nontrivial
     * amount, which affects the accuracy of odometry and control. This ratio represents the
     * number of rotations of the drive motor caused by one rotation of the steer motor.
     *
     * @param ratio Coupled gear ratio between the CANcoder and the drive motor
     * @return this object
     */
    public SwerveModuleConfiguration withCouplingGearRatio(double ratio) {
        this.CouplingGearRatio = ratio;
        return this;
    }

    /**
     * Sets the radius of the drive wheel in meters.
     *
     * @param radius Radius of the drive wheel in meters
     * @return this object
     */
    public SwerveModuleConfiguration withWheelRadiusInMeters(double radius) {
        this.WheelRadiusInMeters = radius;
        return this;
    }

    /**
     * Sets the location of this module's wheel relative to the physical center of the robot in meters.
     *
     * @param moduleLocation Location of this module's wheel relative to the physical center of the robot in meters
     * @return this object
     */
    public SwerveModuleConfiguration withLocation(Translation2d moduleLocation) {
        this.Location = moduleLocation;
        return this;
    }

    /**
     * Sets the steer motor closed loop gains.
     *
     * @param gains Steer motor closed loop gains
     * @return this object
     */
    public SwerveModuleConfiguration withSteerMotorGains(Slot0Configs gains) {
        this.SteerMotorGains = gains;
        return this;
    }

    /**
     * Sets the drive motor closed loop gains.
     *
     * @param gains Drive motor closed loop gains
     * @return this object
     */
    public SwerveModuleConfiguration withDriveMotorGains(Slot0Configs gains) {
        this.DriveMotorGains = gains;
        return this;
    }

    /**
     * Sets the maximum amount of supply current the drive motor.
     *
     * @param supplyCurrent Maximum amount of supply current the drive motor.
     * @return this object
     */
    public SwerveModuleConfiguration withDriveMotorSupplyCurrent(double supplyCurrent) {
        this.DriveMotorSupplyCurrent = supplyCurrent;
        return this;
    }

    /**
     * Sets whether the drive motor supply current limit is enabled.
     *
     * @param enabled True if the drive motor supply current limit is enabled.
     * @return this object
     */
    public SwerveModuleConfiguration withDriveMotorEnableSupplyCurrentLimit(boolean enabled) {
        this.EnableDriveMotorSupplyCurrentLimit = enabled;
        return this;
    }

    /**
     * Sets the maximum amount of stator current the drive motor.
     *
     * @param statorCurrent Maximum amount of stator current the drive motor.
     * @return this object
     */
    public SwerveModuleConfiguration withDriveMotorStatorCurrent(double statorCurrent) {
        this.DriveMotorStatorCurrent = statorCurrent;
        return this;
    }

    /**
     * Sets whether the drive motor stator current limit is enabled.
     *
     * @param enabled True if the drive motor stator current limit is enabled.
     * @return this object
     */
    public SwerveModuleConfiguration withEnableDriveMotorStatorCurrentLimit(boolean enabled) {
        this.EnableDriveMotorStatorCurrentLimit = enabled;
        return this;
    }

    /**
     * Sets whether the drive motor is inverted or not.
     *
     * @param inverted True if the drive motor is inverted.
     * @return this object
     */
    public SwerveModuleConfiguration withDriveMotorInverted(boolean inverted) {
        this.DriveMotorInverted = inverted;
        return this;
    }

    /**
     * Sets the maximum amount of supply current the steer motor.
     *
     * @param supplyCurrent Maximum amount of supply current the steer motor.
     * @return this object
     */
    public SwerveModuleConfiguration withSteerMotorSupplyCurrent(double supplyCurrent) {
        this.SteerMotorSupplyCurrent = supplyCurrent;
        return this;
    }

    /**
     * Sets whether the steer motor supply current limit is enabled.
     *
     * @param enabled True if the steer motor supply current limit is enabled.
     * @return this object
     */
    public SwerveModuleConfiguration withEnableSteerMotorSupplyCurrentLimit(boolean enabled) {
        this.EnableSteerMotorSupplyCurrentLimit = enabled;
        return this;
    }

    /**
     * Sets the maximum amount of stator current the steer motor.
     *
     * @param statorCurrent Maximum amount of stator current the steer motor.
     * @return this object
     */
    public SwerveModuleConfiguration withSteerMotorStatorCurrent(double statorCurrent) {
        this.SteerMotorStatorCurrent = statorCurrent;
        return this;
    }

    /**
     * Sets whether the steer motor stator current limit is enabled.
     *
     * @param enabled True if the steer motor stator current limit is enabled.
     * @return this object
     */
    public SwerveModuleConfiguration withEnableSteerMotorStatorCurrentLimit(boolean enabled) {
        this.EnableSteerMotorStatorCurrentLimit = enabled;
        return this;
    }

    /**
     * Sets whether the steer motor is inverted or not.
     *
     * @param inverted True if the steer motor is inverted.
     * @return this object
     */
    public SwerveModuleConfiguration withSteerMotorInverted(boolean inverted) {
        this.SteerMotorInverted = inverted;
        return this;
    }

    /**
     * When using open-loop drive control, this specifies the speed at which the robot travels
     * when driven with 12 volts, in meters per second. This is used to approximate the output
     * for a desired velocity. If using closed loop control, this value is ignored.
     *
     * @param speed Speed at which the robot travels when driven with c12 volts,
     *              in meters per second
     * @return this object
     */
    public SwerveModuleConfiguration withSpeedAt12VoltsMetersPerSecond(double speed) {
        this.SpeedAt12VoltsMetersPerSecond = speed;
        return this;
    }

    /**
     * Sets how the feedback sensors should be configured.
     * <p>
     * If the robot does not support Pro, then this should remain as RemoteCANcoder.
     * Otherwise, users have the option to use either FusedCANcoder or SyncCANcoder depending
     * on if there is a risk that the CANcoder can fail in a way to provide "good" data.
     *
     * @param source The feedback sensor source
     * @return this object
     */
    public SwerveModuleConfiguration withFeedbackSource(SwerveModuleSteerFeedbackType source) {
        this.FeedbackSource = source;
        return this;
    }

    /**
     * Sets whether to compensate for the cosine of steer motor angle error or not.
     * <p>
     * When compensation is applied, the drive motor speed is scaled based on steer angle error.
     * <p>
     * When the error is 90°, the velocity setpoint should be 0. As the wheel turns
     * towards the setpoint, its velocity should increase. This is achieved by
     * taking the component of the velocity in the direction of the setpoint.
     *
     * @param apply True to apply cosine compensation
     * @return this object
     */
    public SwerveModuleConfiguration withApplyCosineCompensation(boolean apply) {
        this.ApplyCosineCompensation = apply;
        return this;
    }

    /**
     * Sets whether to compensate for the coupling between the drive and steer motors.
     * <p>
     * When compensation is applied, the drive motor position is adjusted based on the
     * steer motor position.
     * <p>
     * When the steer motor rotates, the drive motor rotates a nontrivial amount. This
     * affects the accuracy of odometry and control. This is achieved by
     * taking the component of the position in the direction of the setpoint.
     *
     * @param apply True to apply coupling compensation
     * @return this object
     */
    public SwerveModuleConfiguration withApplyCouplingCompensation(boolean apply) {
        this.ApplyCouplingCompensation = apply;
        return this;
    }

    /**
     * Sets the CanDeviceId for the drive motor.
     *
     * @param id The CanDeviceId
     * @return this object
     */
    public SwerveModuleConfiguration withDriveCanDeviceId(CanDeviceId id) {
        this.DriveCanDeviceId = id;
        return this;
    }

    /**
     * Sets the CanDeviceId for the steer motor.
     *
     * @param id The CanDeviceId
     * @return this object
     */
    public SwerveModuleConfiguration withSteerCanDeviceId(CanDeviceId id) {
        this.SteerCanDeviceId = id;
        return this;
    }

    /**
     * Sets the CanDeviceId for the CANcoder.
     *
     * @param id The CanDeviceId
     * @return this object
     */
    public SwerveModuleConfiguration withCancoderCanDeviceId(CanDeviceId id) {
        this.CancoderCanDeviceId = id;
        return this;
    }
}
