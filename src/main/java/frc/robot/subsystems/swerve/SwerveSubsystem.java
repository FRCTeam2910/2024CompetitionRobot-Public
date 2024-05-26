package frc.robot.subsystems.swerve;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.config.RobotConstants;
import frc.robot.util.*;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
    public static final double ANGULAR_VELOCITY_COEFFICIENT = -0.1;

    /**  The Gyro implementation to read gyro values. */
    private final GyroIO gyroIO;

    /** Gyro IO inputs */
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    /** The swerve modules and positions */
    private final SwerveModule[] swerveModules;

    protected final int updateFrequency;

    protected final double maxVelocityMeterPerSecond;
    protected final double maxAngularVelocityMeterPerSecond;
    protected double lastTimeStamp = Double.NEGATIVE_INFINITY;

    boolean isChassisSpeedsDiscretizeEnabled;
    boolean isSteeringMotionMagicEnabled = false;
    boolean isVelocityLimiterEnabled = false;

    private final RobotConstants constants;

    private final PIDController snapToAnglePID = new PIDController(12.0, 0, 1.0);
    private Rotation2d rotationLock;
    private boolean enableRotationLock;
    private boolean enableTranslationSlowdown;
    private boolean enableRotationSlowdown;
    private double translationCoefficient;
    private double rotationCoefficient;

    // Alerts
    private final Alert gyroDisconnected;

    public SwerveSubsystem(
            RobotConstants constants,
            GyroIO gyroIO,
            SwerveModuleIO frontLeftSwerveModuleIO,
            SwerveModuleIO frontRightSwerveModuleIO,
            SwerveModuleIO backLeftSwerveModuleIO,
            SwerveModuleIO backRightSwerveModuleIO) {

        this.constants = constants;
        updateFrequency = 250; // make this a constant of parameter
        isChassisSpeedsDiscretizeEnabled = constants.getDrivetrainConfiguration().EnableChassisSpeedsDiscretize;
        maxVelocityMeterPerSecond = constants.getDrivetrainConfiguration().MaxSpeedMetersPerSecond;
        maxAngularVelocityMeterPerSecond = constants.getDrivetrainConfiguration().MaxAngularSpeedRadiansPerSecond;

        snapToAnglePID.enableContinuousInput(0.0, 2 * Math.PI);

        this.gyroIO = gyroIO;
        validateCanBusNames(
                gyroIO,
                frontLeftSwerveModuleIO,
                frontRightSwerveModuleIO,
                backLeftSwerveModuleIO,
                backRightSwerveModuleIO);
        swerveModules = new SwerveModule[] {
            new SwerveModule(
                    frontLeftSwerveModuleIO,
                    "FrontLeft",
                    constants.getDrivetrainConfiguration().swerveModuleConfigurations[0]),
            new SwerveModule(
                    frontRightSwerveModuleIO,
                    "FrontRight",
                    constants.getDrivetrainConfiguration().swerveModuleConfigurations[1]),
            new SwerveModule(
                    backLeftSwerveModuleIO,
                    "BackLeft",
                    constants.getDrivetrainConfiguration().swerveModuleConfigurations[2]),
            new SwerveModule(
                    backRightSwerveModuleIO,
                    "BackRight",
                    constants.getDrivetrainConfiguration().swerveModuleConfigurations[3])
        };

        if (Robot.isReal()) {
            Phoenix6Odometry.getInstance().start();
        }

        gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.WARNING);
    }

    @Override
    public void periodic() {
        var odometryTimestamp = 0.0;
        // Thread safe reading of the gyro and swerve inputs.
        Phoenix6Odometry.getInstance().stateLock.readLock().lock();
        try {
            // Assumes Pigeon2 is flat-and-level as latency compensation
            // can only be performed on Z-axis.
            gyroIO.updateInputs(gyroInputs);
            for (var module : swerveModules) {
                module.updateInputs();
                odometryTimestamp = Math.max(odometryTimestamp, module.getOdometryTimestamp());
            }
        } finally {
            Phoenix6Odometry.getInstance().stateLock.readLock().unlock();
        }

        // Log and apply the module periodic updates
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : swerveModules) {
            module.periodic();
        }

        if (Robot.isReal()) {
            gyroDisconnected.set(!gyroInputs.connected);
        }

        if (odometryTimestamp == 0.0) {
            odometryTimestamp = HALUtil.getFPGATime() / 1.0e6;
        }

        // Update odometry in robot state
        Rotation2d yaw = gyroInputs.connected ? Rotation2d.fromRadians(gyroInputs.yaw) : null;
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[swerveModules.length];
        SwerveModuleState[] moduleStates = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            modulePositions[i] = swerveModules[i].getSwerveModulePosition();
            moduleStates[i] = swerveModules[i].getSwerveModuleState();
        }

        SwerveDriveWheelPositions wheelPositions = new SwerveDriveWheelPositions(modulePositions);
        RobotState.getInstance()
                .addOdometryObservation(new RobotState.OdometryObservation(
                        odometryTimestamp,
                        wheelPositions,
                        yaw,
                        moduleStates,
                        gyroAngularVelocity(),
                        gyroInputs.accelerationX,
                        gyroInputs.accelerationY));

        ChassisSpeeds actualChassisSpeeds = RobotState.getInstance().getChassisSpeeds();

        Logger.recordOutput("Drive/ActualChassisSpeed", actualChassisSpeeds);

        Logger.recordOutput("Drive/RotationLockEnabled", enableRotationLock);

        // log outputs
        Logger.recordOutput("Drive/MeasuredModuleStates", modulePositions);
        Logger.recordOutput("Drive/Discretize", isChassisSpeedsDiscretizeEnabled);
        Logger.recordOutput("Drive/VelocityLimiter", isVelocityLimiterEnabled);
    }

    /**
     * Sets the target speed.
     * <p>
     * Make all adjustments to the target speed here such as
     * <ul>
     *     <li>rotational drift compensation </li>
     *     <li>kinematic constraints</li>
     *     <li>etc.</li>
     * </ul>
     *
     * @param chassisSpeed The target {Link ChassisSpeeds} for the drivetrain.
     */
    public void setTargetSpeed(ChassisSpeeds chassisSpeed) {
        // Target speed for the drivetrain as requested by driver or autonomous path follower.
        ChassisSpeeds desiredChassisSpeeds = chassisSpeed;
        if (enableTranslationSlowdown) {
            desiredChassisSpeeds.vxMetersPerSecond *= translationCoefficient;
            desiredChassisSpeeds.vyMetersPerSecond *= translationCoefficient;
        }
        if (enableRotationSlowdown) {
            desiredChassisSpeeds.omegaRadiansPerSecond *= rotationCoefficient;
        }

        //       Compensate for gyro drift by adjusting the target chassis speeds
        var angularVelocity = new Rotation2d(gyroInputs.angularVelocity * ANGULAR_VELOCITY_COEFFICIENT);
        if (angularVelocity.getRadians() != 0.0) {
            desiredChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    chassisSpeed.vxMetersPerSecond,
                    chassisSpeed.vyMetersPerSecond,
                    chassisSpeed.omegaRadiansPerSecond,
                    RobotState.getInstance().getOdometryPose().getRotation());
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    desiredChassisSpeeds,
                    RobotState.getInstance().getOdometryPose().getRotation().plus(angularVelocity));
        }

        ChassisSpeeds adjustedChassisSpeeds = desiredChassisSpeeds;

        // Whether, or not, to compensate for translational skew when translating and rotating
        // DON'T USE Timer.getFPGATimestamp() here. We occasionally get the same value that we've already stored.
        // If dt == 0 ChassisSpeedHelper.discretize() produces invalid values (infinity or NaN).
        // Converts microseconds from HALUtil to seconds
        double timeStamp = HALUtil.getFPGATime() / 1.0e6;
        if (isChassisSpeedsDiscretizeEnabled && (lastTimeStamp > 0.0)) {
            var dt = timeStamp - lastTimeStamp;
            // Compensate for translational skew when translating and rotating a swerve drivetrain
            adjustedChassisSpeeds = ChassisSpeeds.discretize(
                    desiredChassisSpeeds.vxMetersPerSecond,
                    desiredChassisSpeeds.vyMetersPerSecond,
                    desiredChassisSpeeds.omegaRadiansPerSecond,
                    dt);
        }
        // Maintain  the last time stamp
        lastTimeStamp = timeStamp;

        if (enableRotationLock) {
            double rotationalVelocity = MathUtil.clamp(
                    snapToAnglePID.calculate(
                            RobotState.getInstance()
                                    .getOdometryPose()
                                    .getRotation()
                                    .getRadians(),
                            rotationLock.getRadians()),
                    -0.6 * maxAngularVelocityMeterPerSecond,
                    0.6 * maxAngularVelocityMeterPerSecond);
            var fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(
                    adjustedChassisSpeeds,
                    RobotState.getInstance().getOdometryPose().getRotation());
            fieldRel.omegaRadiansPerSecond = rotationalVelocity;
            adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRel, RobotState.getInstance().getOdometryPose().getRotation());
        }

        SwerveModuleState[] setpointStates = constants
                .getDrivetrainConfiguration()
                .Kinematics
                .toSwerveModuleStates(adjustedChassisSpeeds, new Translation2d(Units.inchesToMeters(2.0), 0.0));

        // Re-normalizes the wheel speeds if any individual speed is above the specified
        // maximum accounting for both translational and angular velocities. The states are
        // updated if wheel speeds are above the specified maximum.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                setpointStates,
                adjustedChassisSpeeds,
                getMaxVelocityMeterPerSecond(),
                getMaxVelocityMeterPerSecond(),
                getMaxAngularVelocityMeterPerSecond());

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] =
                    swerveModules[i].setTargetState(setpointStates[i], isSteeringMotionMagicEnabled);
        }

        // Log setpoint states
        Logger.recordOutput(
                "Drive/ChassisTranslationSpeed",
                Math.hypot(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond));
        Logger.recordOutput("Drive/ChassisRotationSpeed", chassisSpeed.omegaRadiansPerSecond);
        Logger.recordOutput("Drive/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SetpointsOptimized", optimizedSetpointStates);
        // Just for logging and comparison.
        RobotState.getInstance().getEstimatedPose();
    }

    public double getMaxVelocityMeterPerSecond() {
        return maxVelocityMeterPerSecond;
    }

    public double getMaxAngularVelocityMeterPerSecond() {
        return maxAngularVelocityMeterPerSecond;
    }

    /**
     * Validates the gyro and swerve modules are on the same can bus.
     *
     * @param gyroIO          The gyro IO.
     * @param swerveModuleIOs The swerve module IOs.
     */
    private void validateCanBusNames(GyroIO gyroIO, SwerveModuleIO... swerveModuleIOs) {
        if (Robot.isReal()) {
            var canBus = gyroIO.getCanDeviceId().getBus();
            for (var swerveModuleIO : swerveModuleIOs) {
                if (!canBus.equals(swerveModuleIO.getCanDeviceId().getBus())) {
                    DriverStation.reportWarning(
                            "Gyro " + gyroIO.getCanDeviceId().toString() + " and swerve module "
                                    + swerveModuleIO.getCanDeviceId().toString() + " have the same CAN bus name: ",
                            false);
                }
            }
        }
    }

    public double gyroAngularVelocity() {
        return gyroInputs.angularVelocity;
    }

    public void setRotationLock(Rotation2d rotationLock) {
        enableRotationLock = true;
        this.rotationLock = rotationLock;
    }

    public void disableRotationLock() {
        enableRotationLock = false;
    }

    public void setSlowdownCoefficient(double translationCoefficient, double rotationCoefficient) {
        this.translationCoefficient = translationCoefficient;
        this.rotationCoefficient = rotationCoefficient;
        enableTranslationSlowdown = true;
        enableRotationSlowdown = true;
    }

    public void disableSlowdownCoefficients() {
        enableTranslationSlowdown = false;
        enableRotationSlowdown = false;
    }

    public boolean isChassisStopped() {
        boolean atZero = true;
        for (SwerveModule swerveModule : swerveModules) {
            if (!MathUtil.isNear(0.0, swerveModule.getVelocityMetersPerSec(), 0.1)) {
                atZero = false;
            }
        }
        return atZero;
    }
}
