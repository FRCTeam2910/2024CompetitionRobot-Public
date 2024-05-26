package frc.robot.config;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.drivers.CanDeviceId;

@SuppressWarnings("UnusedVariable")
public class Typhoon implements RobotConstants {
    private static final String RIO_CANBUS_NAME = "rio";
    private static final String CANIVORE_CANBUS_NAME = "CANivore";

    public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(28);
    public static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(26);

    // Ports and IDs
    private static final CanDeviceId GYRO = new CanDeviceId(2, CANIVORE_CANBUS_NAME);

    // Swerve Modules
    private static final CanDeviceId FRONT_LEFT_DRIVE_MOTOR = new CanDeviceId(1, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_MOTOR = new CanDeviceId(2, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_ENCODER = new CanDeviceId(1, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceId(18, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_MOTOR = new CanDeviceId(17, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_ENCODER = new CanDeviceId(2, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_LEFT_DRIVE_MOTOR = new CanDeviceId(7, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_MOTOR = new CanDeviceId(8, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_ENCODER = new CanDeviceId(3, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_RIGHT_DRIVE_MOTOR = new CanDeviceId(11, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_MOTOR = new CanDeviceId(12, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_ENCODER = new CanDeviceId(4, CANIVORE_CANBUS_NAME);

    // Other Subsystems
    private static final CanDeviceId TURRET = new CanDeviceId(10, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId TURRET_ENCODER_G1 = new CanDeviceId(5, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId TURRET_ENCODER_G2 = new CanDeviceId(6, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId LEFT_SHOOTER = new CanDeviceId(21, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId RIGHT_SHOOTER = new CanDeviceId(23, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId PIVOT = new CanDeviceId(27, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FEEDER_LEFT = new CanDeviceId(24, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FEEDER_TEACUP = new CanDeviceId(9, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FEEDER_RIGHT = new CanDeviceId(26, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId INTAKE_TOP = new CanDeviceId(31, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId INTAKE_BOTTOM = new CanDeviceId(19, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId AMP_BAR = new CanDeviceId(29, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId CLIMBER = new CanDeviceId(30, CANIVORE_CANBUS_NAME);
    private static final int BEAM_BREAK_DIO_ID = 0;

    // ==================================================================================
    // Physical Swerve Module constants
    // ==================================================================================

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry
     * which measures distance as a function of the number of rotations * wheel circumference.
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.92); // Old value 1.95

    /** Ratio between the drive motor shaft and the output shaft the wheel is mounted on. */
    private static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);

    /** Ratio between the steer motor shaft and the steer output shaft. */
    private static final double MK4i_STEER_GEAR_RATIO = 150.0 / 7.0;

    private static final double MK4n_STEER_GEAR_RATIO = 18.75;

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
    private static final double WHEELBASE_LENGTH_METERS = Units.inchesToMeters(20.75);

    /**
     * Wheel track width is the distance between the left and right wheels.
     * Positive y values represent moving towards the left of the robot.
     */
    private static final double WHEEL_TRACK_WIDTH_METERS = Units.inchesToMeters(20.75);

    /**
     * The maximum speed of the robot in meters per second.
     */
    private static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(16.0);

    /**
     * The maximum angular speed of the robot in radians per second.
     * If set to 0, the value is calculated using the max speed in meters per second
     * and the wheelbase radius.
     */
    private static final double MAX_SPEED_IN_RADIANS_PER_SECOND_LIMIT = Units.degreesToRadians(360);

    // CANcoder offsets of the swerve modules
    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = -0.458;
    // -Rotation2d.fromRadians(1.342).getRotations();
    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = -0.454;
    // -Rotation2d.fromRadians(-1.519).getRotations();
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = 0.039;
    // -Rotation2d.fromRadians(-2.586).getRotations();
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = 0.27;
    // -Rotation2d.fromRadians(0.550).getRotations();

    // ==================================================================================
    // Physical Pigeon2 constants
    // ==================================================================================
    private static final int MOUNTING_ANGLE = -90;
    private static final double GYRO_ERROR = 0.28;

    // ==================================================================================
    // Physical Turret constants
    // ==================================================================================

    // (30.0 / 15.0) * (140.0 / 12.0)
    private static final double TURRET_GEAR_RATIO = (28.0 / 12.0) * (32.0 / 16.0) * (140.0 / 12.0);

    // ==================================================================================
    // Path-following constants
    // ==================================================================================
    private static final double FOLLOW_PATH_TRANSLATION_KP = 5.0; // used to be 5.0
    private static final double FOLLOW_PATH_TRANSLATION_KI = 0.0;
    private static final double FOLLOW_PATH_TRANSLATION_KD = 0.0;
    private static final double FOLLOW_PATH_ROTATION_KP = 8.0;
    private static final double FOLLOW_PATH_ROTATION_KI = 0.0;
    private static final double FOLLOW_PATH_ROTATION_KD = 0.0;
    private static final double FOLLOW_PATH_FEEDFORWARD_VELOCITY_COEFFICIENT = 0.0;
    private static final double FOLLOW_PATH_FEEDFORWARD_ACCELERATION_COEFFICIENT = 0.0;

    // CANcoder offsets for the turret
    private static final double G1_OFFSET_ROTATIONS = -Units.degreesToRotations(23.906);
    private static final double G2_OFFSET_ROTATIONS = -Units.degreesToRotations(286.787);
    private static final boolean G1_INVERTED = false;
    private static final boolean G2_INVERTED = true;

    // Robot configuration
    private final PortConfiguration portConfiguration;
    private final DrivetrainConfiguration drivetrainConfiguration;
    private final Pigeon2Constants pigeon2Constants;

    // ==================================================================================
    // Physical Limelight Offsets
    // ==================================================================================
    private final LimelightConfiguration limelightConfiguration;

    private final TurretConfiguration turretConfiguration;

    private final FollowPathConfiguration followPathConfiguration;

    public Typhoon() {
        // Initialize PortConfiguration
        portConfiguration = new PortConfiguration().withCANBus(CANIVORE_CANBUS_NAME);
        portConfiguration.intakeBottomMotorID = INTAKE_BOTTOM;
        portConfiguration.intakeTopMotorID = INTAKE_TOP;
        portConfiguration.teacupMotorCanDeviceID = FEEDER_TEACUP;
        portConfiguration.shooterLeftMotorID = LEFT_SHOOTER;
        portConfiguration.shooterRightMotorID = RIGHT_SHOOTER;
        portConfiguration.leftFeederMotorID = FEEDER_RIGHT;
        portConfiguration.rightFeederMotorID = FEEDER_LEFT;
        portConfiguration.beamBreakDIOId = BEAM_BREAK_DIO_ID;
        portConfiguration.pivotID = PIVOT;
        portConfiguration.ampBarID = AMP_BAR;
        portConfiguration.climberID = CLIMBER;
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
                .withEnableChassisSpeedsDiscretize(false) // changed this as test TODO REMOVE
                // In order of front left, front right, back left, back right
                .withSwerveDriveKinematics(new SwerveDriveKinematics(
                        new Translation2d(WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2),
                        new Translation2d(WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2),
                        new Translation2d(-WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2),
                        new Translation2d(-WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2)));

        // In order of front left, front right, back left, back right
        drivetrainConfiguration.swerveModuleConfigurations = new SwerveModuleConfiguration[] {
            new SwerveModuleConfiguration()
                    .withName("Front Left")
                    .withDriveCanDeviceId(FRONT_LEFT_DRIVE_MOTOR)
                    .withSteerCanDeviceId(FRONT_LEFT_STEER_MOTOR)
                    .withCancoderCanDeviceId(FRONT_LEFT_STEER_ENCODER)
                    .withCANcoderOffsetRotations(FRONT_LEFT_STEER_OFFSET_ROTATIONS)
                    .withFeedbackSource(SwerveModuleConfiguration.SwerveModuleSteerFeedbackType.FusedCANcoder)
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(MK4n_STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                    .withDriveMotorInverted(true)
                    .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                    .withSteerMotorInverted(true)
                    .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                    .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                    .withLocation(new Translation2d(WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2))
                    .withSpeedAt12VoltsMetersPerSecond(drivetrainConfiguration.MaxSpeedMetersPerSecond),
            new SwerveModuleConfiguration()
                    .withName("Front Right")
                    .withDriveCanDeviceId(FRONT_RIGHT_DRIVE_MOTOR)
                    .withSteerCanDeviceId(FRONT_RIGHT_STEER_MOTOR)
                    .withCancoderCanDeviceId(FRONT_RIGHT_STEER_ENCODER)
                    .withCANcoderOffsetRotations(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
                    .withFeedbackSource(SwerveModuleConfiguration.SwerveModuleSteerFeedbackType.FusedCANcoder)
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(MK4n_STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                    .withDriveMotorInverted(true)
                    .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                    .withSteerMotorInverted(true)
                    .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                    .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                    .withLocation(new Translation2d(WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2))
                    .withSpeedAt12VoltsMetersPerSecond(drivetrainConfiguration.MaxSpeedMetersPerSecond),
            new SwerveModuleConfiguration()
                    .withName("Back Left")
                    .withDriveCanDeviceId(BACK_LEFT_DRIVE_MOTOR)
                    .withSteerCanDeviceId(BACK_LEFT_STEER_MOTOR)
                    .withCancoderCanDeviceId(BACK_LEFT_STEER_ENCODER)
                    .withCANcoderOffsetRotations(BACK_LEFT_STEER_OFFSET_ROTATIONS)
                    .withFeedbackSource(SwerveModuleConfiguration.SwerveModuleSteerFeedbackType.FusedCANcoder)
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(MK4i_STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                    .withDriveMotorInverted(true)
                    .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                    .withSteerMotorInverted(true)
                    .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                    .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                    .withLocation(new Translation2d(-WHEELBASE_LENGTH_METERS / 2, WHEEL_TRACK_WIDTH_METERS / 2))
                    .withSpeedAt12VoltsMetersPerSecond(drivetrainConfiguration.MaxSpeedMetersPerSecond),
            new SwerveModuleConfiguration()
                    .withName("Back Right")
                    .withDriveCanDeviceId(BACK_RIGHT_DRIVE_MOTOR)
                    .withSteerCanDeviceId(BACK_RIGHT_STEER_MOTOR)
                    .withCancoderCanDeviceId(BACK_RIGHT_STEER_ENCODER)
                    .withCANcoderOffsetRotations(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
                    .withFeedbackSource(SwerveModuleConfiguration.SwerveModuleSteerFeedbackType.FusedCANcoder)
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(MK4i_STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                    .withDriveMotorInverted(true)
                    .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                    .withSteerMotorInverted(true)
                    .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                    .withWheelRadiusInMeters(WHEEL_RADIUS_METERS)
                    .withLocation(new Translation2d(-WHEELBASE_LENGTH_METERS / 2, -WHEEL_TRACK_WIDTH_METERS / 2))
                    .withSpeedAt12VoltsMetersPerSecond(drivetrainConfiguration.MaxSpeedMetersPerSecond),
        };

        pigeon2Constants = new Pigeon2Constants()
                .withCanDeviceId(GYRO)
                .withMountPoseYaw(MOUNTING_ANGLE)
                .withGyroScalarZ(GYRO_ERROR);

        var turretTalonFXConstants = new ServoMotorConfiguration.TalonFXConstants()
                .withCanDeviceId(new CanDeviceId(TURRET.getDeviceNumber(), TURRET.getBus()));

        var turretMotorConfiguration = new ServoMotorConfiguration()
                .withName("Turret")
                .withMasterConstants(turretTalonFXConstants)
                .withRotationsPerUnitDistance(TURRET_GEAR_RATIO / 360.0)
                .withSensorToMechanismRatio(TURRET_GEAR_RATIO)
                .withMinUnitsLimit(-360.0)
                .withMaxUnitsLimit(360.0)
                .withSoftLimitDeadband(0.05)
                .withNeutralMode(NeutralModeValue.Brake)
                .withPositionKp(1.0)
                .withPositionKd(0.15)
                .withSupplyCurrentLimit(100)
                .withEnableSupplyCurrentLimit(true)
                .withStatorCurrentLimit(40)
                .withEnableStatorCurrentLimit(true)
                .withRampRate(0.02);

        turretConfiguration = new TurretConfiguration()
                .withTurretMotor(turretMotorConfiguration)
                .withCanCoderG1CanDeviceId(TURRET_ENCODER_G1)
                .withCanCoderG2CanDeviceId(TURRET_ENCODER_G2)
                .withEncodeOffserG1Rotations(G1_OFFSET_ROTATIONS)
                .withEncodeOffserG2ROtations(G2_OFFSET_ROTATIONS)
                .withG1Inverted(G1_INVERTED)
                .withG2Inverted(G2_INVERTED);

        // TODO: Configure with 2024 Limelight Position Measurements
        limelightConfiguration = new LimelightConfiguration()
                .withHeightOffset(Units.inchesToMeters(8.5))
                .withMountingPitch(Units.degreesToRadians(26.79));

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
