package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

@SuppressWarnings("InconsistentCapitalization")
public class DrivetrainConfiguration {
    public double wheelRadiusInMeters = 0.0;

    /** The locations of the swerve modules on the robot. */
    public Translation2d[] swerveModuleLocations;

    /** The offsets of the swerve modules on the robot as read by CANCoder. */
    public double[] swerveModuleOffsets;

    /** The name of the CAN bus to use for the swerve modules. Default is "rio" meaning no CAN-FD bus. */
    public String CANbusName = "rio";

    /** Whether the swerve modules support Phoenix 6 PRO mode which enables FOC controls. */
    public boolean SupportsPro = false;

    /** Whether to enable the discretization of chassis speeds using WPILibs method to
     * compensate for translational skew when translating and rotating.
     */
    public boolean EnableChassisSpeedsDiscretize = false;

    /** The maximum speed of the robot in meters per second */
    public double MaxSpeedMetersPerSecond = 1.0;

    /** The maximum acceleration of the robot in meters per second squared */
    public double MaxAccelerationMetersPerSecondSquared = 1.0;

    /** The maximum angular speed of the robot in radians per second */
    public double MaxAngularSpeedRadiansPerSecond = 1.0;

    /** The maximum centripetal acceleration of the robot in meters per second squared */
    public double MaxCentripetalAccelerationInMetersPerSecondSquared = 1.0;

    public SwerveDriveKinematics Kinematics;

    public SwerveModuleConfiguration[] swerveModuleConfigurations = new SwerveModuleConfiguration[4];

    public DrivetrainConfiguration withMaxSpeedMetersPerSecond(double maxSpeedMetersPerSecond) {
        this.MaxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        return this;
    }

    public DrivetrainConfiguration withMaxAccelerationMetersPerSecondSquared(
            double maxAccelerationMetersPerSecondSquared) {
        this.MaxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
        return this;
    }

    public DrivetrainConfiguration withMaxAngularSpeedRadiansPerSecond(double maxAngularSpeedRadiansPerSecond) {
        this.MaxAngularSpeedRadiansPerSecond = maxAngularSpeedRadiansPerSecond;
        return this;
    }

    public DrivetrainConfiguration withMaxCentriptalAccelerationInMetersPerSecondSquared(
            double maxCentriptalAccelerationInMetersPerSecondSquared) {
        this.MaxCentripetalAccelerationInMetersPerSecondSquared = maxCentriptalAccelerationInMetersPerSecondSquared;
        return this;
    }

    public DrivetrainConfiguration withWheelRadiusInMeters(double wheelRadiusInMeters) {
        this.wheelRadiusInMeters = wheelRadiusInMeters;
        return this;
    }

    public DrivetrainConfiguration withSupportsPro(boolean supportsPro) {
        this.SupportsPro = supportsPro;
        return this;
    }

    public DrivetrainConfiguration withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }

    public DrivetrainConfiguration withSwerveModuleConfigurations(
            SwerveModuleConfiguration[] swerveModuleConfigurations) {
        this.swerveModuleConfigurations = swerveModuleConfigurations;
        return this;
    }

    public DrivetrainConfiguration withSwerveModuleLocations(Translation2d[] locations) {
        this.swerveModuleLocations = locations;
        return this;
    }

    public DrivetrainConfiguration withSwerveModuleOffsets(double[] offsets) {
        this.swerveModuleOffsets = offsets;
        return this;
    }

    public DrivetrainConfiguration withEnableChassisSpeedsDiscretize(boolean enableChassisSpeedsDiscretize) {
        this.EnableChassisSpeedsDiscretize = enableChassisSpeedsDiscretize;
        return this;
    }

    public DrivetrainConfiguration withSwerveDriveKinematics(SwerveDriveKinematics kinematics) {
        this.Kinematics = kinematics;
        return this;
    }
}
