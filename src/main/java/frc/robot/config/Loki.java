package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.drivers.CanDeviceId;

@SuppressWarnings("UnusedVariable")
public class Loki implements RobotConstants {
    private static final String CANIVORE_CANBUS_NAME = "CANivore";

    // Ports and IDs
    private static final CanDeviceId GYRO = new CanDeviceId(1, CANIVORE_CANBUS_NAME);

    // Swerve Modules
    private static final CanDeviceId FRONT_LEFT_DRIVE_MOTOR = new CanDeviceId(7, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_MOTOR = new CanDeviceId(8, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_ENCODER = new CanDeviceId(4, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceId(3, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_MOTOR = new CanDeviceId(4, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_ENCODER = new CanDeviceId(3, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_LEFT_DRIVE_MOTOR = new CanDeviceId(5, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_MOTOR = new CanDeviceId(6, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_ENCODER = new CanDeviceId(2, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_RIGHT_DRIVE_MOTOR = new CanDeviceId(1, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_MOTOR = new CanDeviceId(2, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_ENCODER = new CanDeviceId(1, CANIVORE_CANBUS_NAME);

    // ==================================================================================
    // Physical Swerve Module constants
    // ==================================================================================

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry
     * which measures distance as a function of the number of rotations * wheel circumference.
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.95);

    /**
     * Ratio between the drive motor shaft and the output shaft the wheel is mounted on.
     */
    private static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);

    /**
     * Ratio between the steer motor shaft and the steer output shaft.
     */
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    /**
     * The coupled gear ratio between the CanCoder and the drive motor.
     * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
     */
    private static final double COUPLING_GEAR_RATIO = 0.0;

    // ==================================================================================
    // Physical Drivetrain constants
    // ==================================================================================

    /**
     * Wheelbase length is the distance between the front and back wheels.
     * Positive x values represent moving towards the front of the robot
     */
    private static final double WHEELBASE_LENGTH_METERS = Units.inchesToMeters(22.75);

    /**
     * Wheel track width is the distance between the left and right wheels.
     * Positive y values represent moving towards the left of the robot.
     */
    private static final double WHEEL_TRACK_WIDTH_METERS = Units.inchesToMeters(20.75);

    public static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(26);
    public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(28);

    /**
     * The maximum speed of the robot in meters per second.
     */
    private static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(17.2);

    /**
     * The maximum angular speed of the robot in radians per second.
     * If set to 0, the value is calculated using the max speed in meters per second
     * and the wheelbase radius.
     */

    // CANcoder offsets of the swerve modules
    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = -Units.degreesToRotations(248.81 - 180.0);

    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = -Units.degreesToRotations(122.51 + 180.0);
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = -Units.degreesToRotations(49.39 + 180.0);
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = -Units.degreesToRotations(65.21 + 180.0);

    // ==================================================================================
    // Physical Pigeon2 constants
    // ==================================================================================

    private static final int MOUNTING_ANGLE = 90;
    private static final double GYRO_ERROR = 1.6;

    // ==================================================================================
    // Path-following constants
    // ==================================================================================
    private static final double FOLLOW_PATH_TRANSLATION_KP = 5.0;
    private static final double FOLLOW_PATH_TRANSLATION_KI = 0.0;
    private static final double FOLLOW_PATH_TRANSLATION_KD = 0.0;
    private static final double FOLLOW_PATH_ROTATION_KP = 5.0;
    private static final double FOLLOW_PATH_ROTATION_KI = 0.0;
    private static final double FOLLOW_PATH_ROTATION_KD = 0.0;
    private static final double FOLLOW_PATH_FEEDFORWARD_VELOCITY_COEFFICIENT = 0.0;
    private static final double FOLLOW_PATH_FEEDFORWARD_ACCELERATION_COEFFICIENT = 0.0;

    // Robot configuration
    private final PortConfiguration portConfiguration;
    private final DrivetrainConfiguration drivetrainConfiguration;
    private final Pigeon2Constants pigeon2Constants;

    private final TurretConfiguration turretConfiguration;
    private final ServoMotorConfiguration pivotConfiguration;

    // ==================================================================================
    // Physical Limelight Offsets
    // ==================================================================================
    private final LimelightConfiguration limelightConfiguration;

    private final FollowPathConfiguration followPathConfiguration;

    public Loki() {
        portConfiguration = new PortConfiguration().withCANBus(CANIVORE_CANBUS_NAME);

        // Calculate constraints
        var maxAccelerationMetersPerSecondSquared = MAX_SPEED_METERS_PER_SECOND / 0.5;
        var radiusInMeters = Math.hypot(WHEEL_TRACK_WIDTH_METERS / 2, WHEELBASE_LENGTH_METERS / 2);
        var maxAngularSpeedRadiansPerSecond = MAX_SPEED_METERS_PER_SECOND / radiusInMeters;
        var maxCentripetalAccelerationInMetersPerSecondSquared =
                Math.pow(MAX_SPEED_METERS_PER_SECOND, 2) / radiusInMeters;
        // Initialize DrivetrainConfiguration
        drivetrainConfiguration = new DrivetrainConfiguration()
                .withCANbusName(portConfiguration.CANBus)
                .withSupportsPro(true)
                .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                .withMaxSpeedMetersPerSecond(MAX_SPEED_METERS_PER_SECOND)
                .withMaxAccelerationMetersPerSecondSquared(maxAccelerationMetersPerSecondSquared)
                .withMaxAngularSpeedRadiansPerSecond(maxAngularSpeedRadiansPerSecond)
                .withMaxCentriptalAccelerationInMetersPerSecondSquared(
                        maxCentripetalAccelerationInMetersPerSecondSquared)
                // In order of front left, front right, back left, back right
                .withSwerveModuleLocations(new Translation2d[] {
                    new Translation2d(WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2),
                    new Translation2d(WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2),
                    new Translation2d(-WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2),
                    new Translation2d(-WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2)
                })
                .withSwerveModuleOffsets(new double[] {
                    FRONT_LEFT_STEER_OFFSET_ROTATIONS,
                    FRONT_RIGHT_STEER_OFFSET_ROTATIONS,
                    BACK_LEFT_STEER_OFFSET_ROTATIONS,
                    BACK_RIGHT_STEER_OFFSET_ROTATIONS
                })
                .withEnableChassisSpeedsDiscretize(true)
                // In order of front left, front right, back left, back right
                .withSwerveDriveKinematics(new SwerveDriveKinematics(
                        new Translation2d(WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2),
                        new Translation2d(WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2),
                        new Translation2d(-WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2),
                        new Translation2d(-WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2)));
        ;

        // In order of front left, front right, back left, back right
        drivetrainConfiguration.swerveModuleConfigurations = new SwerveModuleConfiguration[] {
            new SwerveModuleConfiguration()
                    .withName("Front Left")
                    .withDriveCanDeviceId(FRONT_LEFT_DRIVE_MOTOR)
                    .withSteerCanDeviceId(FRONT_LEFT_STEER_MOTOR)
                    .withCancoderCanDeviceId(FRONT_LEFT_STEER_ENCODER)
                    .withCANcoderOffsetRotations(drivetrainConfiguration.swerveModuleOffsets[0])
                    .withFeedbackSource(SwerveModuleConfiguration.SwerveModuleSteerFeedbackType.FusedCANcoder)
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                    .withDriveMotorInverted(true)
                    .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                    .withSteerMotorInverted(true)
                    .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                    .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                    .withLocation(new Translation2d(WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2))
                    .withSpeedAt12VoltsMetersPerSecond(drivetrainConfiguration.MaxSpeedMetersPerSecond)
                    .withApplyCosineCompensation(false),
            new SwerveModuleConfiguration()
                    .withName("Front Right")
                    .withDriveCanDeviceId(FRONT_RIGHT_DRIVE_MOTOR)
                    .withSteerCanDeviceId(FRONT_RIGHT_STEER_MOTOR)
                    .withCancoderCanDeviceId(FRONT_RIGHT_STEER_ENCODER)
                    .withCANcoderOffsetRotations(drivetrainConfiguration.swerveModuleOffsets[1])
                    .withFeedbackSource(SwerveModuleConfiguration.SwerveModuleSteerFeedbackType.FusedCANcoder)
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                    .withDriveMotorInverted(true)
                    .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                    .withSteerMotorInverted(true)
                    .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                    .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                    .withLocation(new Translation2d(WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2))
                    .withSpeedAt12VoltsMetersPerSecond(drivetrainConfiguration.MaxSpeedMetersPerSecond)
                    .withApplyCosineCompensation(false),
            new SwerveModuleConfiguration()
                    .withName("Back Left")
                    .withDriveCanDeviceId(BACK_LEFT_DRIVE_MOTOR)
                    .withSteerCanDeviceId(BACK_LEFT_STEER_MOTOR)
                    .withCancoderCanDeviceId(BACK_LEFT_STEER_ENCODER)
                    .withCANcoderOffsetRotations(drivetrainConfiguration.swerveModuleOffsets[2])
                    .withFeedbackSource(SwerveModuleConfiguration.SwerveModuleSteerFeedbackType.FusedCANcoder)
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                    .withDriveMotorInverted(true)
                    .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                    .withSteerMotorInverted(true)
                    .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                    .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                    .withLocation(new Translation2d(-WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2))
                    .withSpeedAt12VoltsMetersPerSecond(drivetrainConfiguration.MaxSpeedMetersPerSecond)
                    .withApplyCosineCompensation(false),
            new SwerveModuleConfiguration()
                    .withName("Back Right")
                    .withDriveCanDeviceId(BACK_RIGHT_DRIVE_MOTOR)
                    .withSteerCanDeviceId(BACK_RIGHT_STEER_MOTOR)
                    .withCancoderCanDeviceId(BACK_RIGHT_STEER_ENCODER)
                    .withCANcoderOffsetRotations(drivetrainConfiguration.swerveModuleOffsets[3])
                    .withFeedbackSource(SwerveModuleConfiguration.SwerveModuleSteerFeedbackType.FusedCANcoder)
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                    .withDriveMotorInverted(true)
                    .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                    .withSteerMotorInverted(true)
                    .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                    .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                    .withLocation(new Translation2d(-WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2))
                    .withSpeedAt12VoltsMetersPerSecond(drivetrainConfiguration.MaxSpeedMetersPerSecond)
                    .withApplyCosineCompensation(false),
        };

        pigeon2Constants = new Pigeon2Constants()
                .withCanDeviceId(GYRO)
                .withMountPoseYaw(MOUNTING_ANGLE)
                .withGyroScalarZ(GYRO_ERROR);

        turretConfiguration = new TurretConfiguration();
        pivotConfiguration = new ServoMotorConfiguration();

        limelightConfiguration = new LimelightConfiguration()
                .withMountingRoll(Math.toRadians(-6))
                .withHeightOffset(Units.inchesToMeters(25))
                .withLengthOffset(Units.inchesToMeters(10))
                .withWidthOffset(Units.inchesToMeters(1.5));

        followPathConfiguration = new FollowPathConfiguration()
                .withTranslationKp(FOLLOW_PATH_TRANSLATION_KP)
                .withTranslationKi(FOLLOW_PATH_TRANSLATION_KI)
                .withTranslationKd(FOLLOW_PATH_TRANSLATION_KD)
                .withRotationKp(FOLLOW_PATH_ROTATION_KP)
                .withRotationKi(FOLLOW_PATH_ROTATION_KI)
                .withRotationKd(FOLLOW_PATH_ROTATION_KD)
                .withFeedforwardVelocityCoefficient(FOLLOW_PATH_FEEDFORWARD_VELOCITY_COEFFICIENT)
                .withFeedforwardAccelerationCoefficient(FOLLOW_PATH_FEEDFORWARD_ACCELERATION_COEFFICIENT);
    }

    @Override
    public DrivetrainConfiguration getDrivetrainConfiguration() {
        return drivetrainConfiguration;
    }

    @Override
    public PortConfiguration getPortConfiguration() {
        return portConfiguration;
    }

    @Override
    public Pigeon2Constants getPigeon2Constants() {
        return pigeon2Constants;
    }

    @Override
    public TurretConfiguration getTurretConfiguration() {
        return turretConfiguration;
    }

    @Override
    public LimelightConfiguration getLimelightConfiguration() {
        return limelightConfiguration;
    }

    @Override
    public FollowPathConfiguration getFollowPathConfiguration() {
        return followPathConfiguration;
    }
}
