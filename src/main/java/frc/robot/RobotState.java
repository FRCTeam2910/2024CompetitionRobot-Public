package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FieldConstants;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotIdentity;
import frc.robot.subsystems.CompensationCoefficientConstants;
import frc.robot.util.Angles;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.config.FieldConstants.BLUE_SPEAKER;
import static frc.robot.config.FieldConstants.RED_SPEAKER;

@SuppressWarnings("UnusedVariable")
public class RobotState {
    private static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

    public record OdometryObservation(
            double timestamp,
            SwerveDriveWheelPositions wheelPositions,
            Rotation2d gyroAngle,
            SwerveModuleState[] moduleStates,
            double gyroAngularVelocity,
            double accelerationX,
            double accelerationY) {}

    /**
     * Represents an observation of where the camera is on the field as determined by an april tag.
     */
    public record VisionFromAprilTagObservation(
            double timestamp, Pose2d visionPose, int tagId, boolean hasTarget, double horizontalDistance) {}

    public record TurretAngleObservation(double timestamp, Rotation2d turretAngle) {}

    public record AimingParameters(
            Rotation2d turretAimingAngle,
            Rotation2d turretAimingOffset,
            Translation2d effectiveDistance,
            double driveFeedVelocity) {}

    protected AimingParameters latestParameters = null;

    private static RobotState instance;

    // Pose Estimation Members
    private Pose2d odometryPose = new Pose2d();

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SECONDS);

    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
            TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SECONDS);

    private Optional<Pose2d> speakerCameraPose = Optional.empty();
    private double speakerHorizontalDistance = Double.NaN;
    private Optional<Pose2d> robotPoseFromCameraPose = Optional.empty();
    private double visionHorizontalDistance = 0.0;

    private boolean hasTarget = false;
    private boolean hasPiece = false;
    private double lastGyroAngularVelocity = 0.0;
    private double lastAccelerationXFromPigeon = 0.0;
    private double lastAccelerationYFromPigeon = 0.0;

    protected Twist2d robotAccelerations = new Twist2d();
    protected SwerveDriveOdometry odometry;

    private SwerveDriveWheelPositions lastWheelPositions = new SwerveDriveWheelPositions(new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    });

    private SwerveModuleState[] moduleStates = new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };

    private Rotation2d lastGyroAngle = new Rotation2d();
    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
    private final SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private RobotState() {
        kinematics =
                RobotConstants.getRobotConstants(RobotIdentity.getIdentity()).getDrivetrainConfiguration().Kinematics;
        odometry = new SwerveDriveOdometry(
                kinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                new Pose2d());
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                new Pose2d(),
                VecBuilder.fill(Units.inchesToMeters(2.0), Units.inchesToMeters(2.0), Units.degreesToRadians(2.0)),
                VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(1.0), Units.degreesToRadians(2.0)));

        turretAngleBuffer.addSample(-3.0, new Rotation2d());
    }

    public synchronized void addOdometryObservation(OdometryObservation observation) {
        latestParameters = null;
        if (observation.gyroAngle != null) {
            lastGyroAngle = observation.gyroAngle;
            lastGyroAngularVelocity = observation.gyroAngularVelocity;
        } else {
            // If gyro is not connected, simulate gyro using wheel position deltas
            Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions);
            lastGyroAngle = lastGyroAngle.plus(new Rotation2d(twist.dtheta));
            // simulate gyro drift.  +/- 0.25 degree.
            // var drift = Rotation2d.fromDegrees(0.0); // Rotation2d.fromDegrees(-0.25 + (Math.random() * 0.5));
            // lastGyroAngle = lastGyroAngle.plus(drift);
            lastGyroAngularVelocity = twist.dtheta;
        }

        lastWheelPositions = observation.wheelPositions;
        // update module states
        moduleStates = observation.moduleStates;
        var chassisSpeeds = kinematics.toChassisSpeeds(observation.moduleStates);
        robotAccelerations = new Twist2d(
                (chassisSpeeds.vxMetersPerSecond - lastChassisSpeeds.vxMetersPerSecond) / observation.timestamp,
                (chassisSpeeds.vyMetersPerSecond - lastChassisSpeeds.vyMetersPerSecond) / observation.timestamp,
                (chassisSpeeds.omegaRadiansPerSecond - lastChassisSpeeds.omegaRadiansPerSecond)
                        / observation.timestamp);
        lastChassisSpeeds = chassisSpeeds;
        // update odometry
        odometryPose = odometry.update(lastGyroAngle, observation.wheelPositions);
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp, odometryPose);
        // update pose estimator
        poseEstimator.updateWithTime(observation.timestamp, lastGyroAngle, observation.wheelPositions);

        lastAccelerationXFromPigeon = observation.accelerationX;
        lastAccelerationYFromPigeon = observation.accelerationY;
    }

    /**
     * Adds an observation of a camera relative pose to an april tag.
     * Adds a robot pose (derived from transforming the camera pose to the center of the robot) to the {@link SwerveDrivePoseEstimator} for improved odometry.
     *
     * @param observation The observation to add.
     */
    public synchronized void addVisionFromAprilTagObservation(VisionFromAprilTagObservation observation) {
        latestParameters = null;
        hasTarget = observation.hasTarget;

        visionHorizontalDistance = observation.horizontalDistance;
        speakerHorizontalDistance = Double.NaN;
        speakerCameraPose = Optional.empty();
        Optional<Pose2d> visionPose = Optional.empty();

        if (observation.visionPose != null) {
            if (observation.tagId == BLUE_SPEAKER || observation.tagId == RED_SPEAKER) {
                speakerCameraPose = Optional.of(observation.visionPose);
                speakerHorizontalDistance = observation.horizontalDistance;
            }

            // Get the difference between current and previous odometry pose.
            var previousRobotPose = poseBuffer.getSample(observation.timestamp).orElse(odometryPose);

            // Get the difference between current and previous turret angle.
            var turretAngle = getCurrentTurretAngle();
            var previousTurretAngle =
                    turretAngleBuffer.getSample(observation.timestamp).orElse(turretAngle);

            var newCameraPose = latencyCompensateVision(
                    observation.visionPose, previousRobotPose, odometryPose, previousTurretAngle, turretAngle);

            // Recalculate distance to account for robot movement.
            var poseVisionHorizontalDistance =
                    observation.visionPose.getTranslation().getNorm();
            Logger.recordOutput("Vision/HorizontalDistanceFromPose", poseVisionHorizontalDistance);

            var latencyCompensatedVisionHorizontalDistance =
                    newCameraPose.getTranslation().getNorm();
            Logger.recordOutput(
                    "Vision/LatencyCompensatedHorizontalDistance", latencyCompensatedVisionHorizontalDistance);
        }

        robotPoseFromCameraPose = getRobotToField(observation);

        // Just for comparison, add the robot pose to the pose estimator if present.
        robotPoseFromCameraPose.ifPresent(pose2d -> poseEstimator.addVisionMeasurement(pose2d, observation.timestamp));
    }

    /**
     * Applies latency compensation to a vision observation.
     *
     * @param cameraPose          The camera pose.
     * @param previousRobotPose   The previous robot pose.
     * @param currentRobotPose    The current robot pose.
     * @param previousTurretAngle The previous turret angle.
     * @param currentTurretAngle  The current turret angle.
     * @return The latency compensated vision pose.
     */
    protected Pose2d latencyCompensateVision(
            Pose2d cameraPose,
            Pose2d previousRobotPose,
            Pose2d currentRobotPose,
            Rotation2d previousTurretAngle,
            Rotation2d currentTurretAngle) {
        // Get the relative movement of the robot since the camera observation.
        var robotPoseChange = currentRobotPose.minus(previousRobotPose);
        // Get the turret angle change since the camera observation.
        var turretAngleChange = currentTurretAngle.minus(previousTurretAngle);

        // Apply the changes to the camera pose to get the current camera pose.
        // If the turret angle changes, the camera pose will change.
        return new Pose2d(
                cameraPose.getTranslation().plus(robotPoseChange.getTranslation()),
                cameraPose.getRotation().plus(turretAngleChange));
    }

    @AutoLogOutput(key = "Vision/CameraPose")
    public synchronized Pose2d getSpeakerCameraPose() {
        return speakerCameraPose.orElse(null);
    }

    public synchronized double getLastGyroAngularVelocity() {
        return lastGyroAngularVelocity;
    }

    private Optional<Pose2d> getRobotToField(VisionFromAprilTagObservation observation) {
        if (observation.visionPose == null) {
            return Optional.empty();
        }
        return getRobotToField(
                observation.visionPose,
                FieldConstants.FIELD_LAYOUT.getTagPose(observation.tagId).get(),
                turretAngleBuffer.getSample(observation.timestamp).orElse(getCurrentTurretAngle()),
                odometryPose.getRotation());
    }

    /**
     * Returns the field relative robot pose.
     *
     * @param visionPose       The vision pose.
     * @param tag              The april tag pose.
     * @param turretAngle      The turret angle.
     * @param robotOrientation The robot orientation.
     * @return The field relative robot pose.
     */
    protected Optional<Pose2d> getRobotToField(
            Pose2d visionPose, Pose3d tag, Rotation2d turretAngle, Rotation2d robotOrientation) {

        // Remove the inherit rotation in the apriltag pose.
        var tag2d = new Pose2d(tag.toPose2d().getTranslation(), new Rotation2d());
        var totalAngle = visionPose.getRotation().plus(turretAngle).plus(robotOrientation);

        // visionPose is the pose of the camera relative to the tag.
        var tagToCamera = new Pose2d(visionPose.getTranslation(), new Rotation2d())
                .rotateBy(totalAngle)
                .rotateBy(Rotation2d.fromDegrees(180));

        var tagToCameraTransform = new Transform2d(tagToCamera.getTranslation(), new Rotation2d());
        var cameraToTag = tag2d.plus(tagToCameraTransform);

        // Calculate the root relative offset of camera.
        var turretToCamera = new Translation2d(Units.inchesToMeters(8.979), 0);
        var robotToTurret = new Translation2d(Units.inchesToMeters(-4.0), 0);
        var robotToCamera = robotToTurret.plus(turretToCamera.rotateBy(turretAngle.plus(visionPose.getRotation())));

        var robotToTagTranslation = robotToCamera
                .rotateBy(robotOrientation.rotateBy(Rotation2d.fromDegrees(180)))
                .plus(cameraToTag.getTranslation());

        var robotToTag = new Pose2d(robotToTagTranslation, robotOrientation);
        return Optional.of(robotToTag);
    }

    protected synchronized Optional<Pose2d> getRobotPoseFromCameraPose(
            Optional<Pose2d> cameraPose,
            Pose2d odometryPose,
            Rotation2d turretAngle,
            Pose3d tag,
            double visionHorizontalDistance) {
        if (cameraPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d tagCoords = tag;
        Rotation2d fieldRelativeRobotOrientation = odometryPose.getRotation();
        Rotation2d totalCameraAngle =
                cameraPose.get().getRotation().plus(turretAngle).plus(fieldRelativeRobotOrientation);

        Logger.recordOutput("Vision/TotalCameraAngle", totalCameraAngle);

        Rotation2d aprilTagToCameraLensAngle = totalCameraAngle.minus(Rotation2d.fromDegrees(180));

        double fieldRelativeAprilTagToCameraX = visionHorizontalDistance * aprilTagToCameraLensAngle.getCos();
        double fieldRelativeAprilTagToCameraY = visionHorizontalDistance * aprilTagToCameraLensAngle.getSin();

        Logger.recordOutput("Vision/AprilTagToCameraX", fieldRelativeAprilTagToCameraX);
        Logger.recordOutput("Vision/AprilTagToCameraY", fieldRelativeAprilTagToCameraY);

        double radiusFromCameraLensToTurretCenterMeters = Units.inchesToMeters(8.979);

        // Field Oriented Limelight Rotation
        Rotation2d fieldRelativeLimelightRotation =
                Rotation2d.fromDegrees(180).minus(totalCameraAngle.plus(fieldRelativeRobotOrientation));

        double fieldRelativeCameraToTurretCenterX =
                radiusFromCameraLensToTurretCenterMeters * fieldRelativeLimelightRotation.getCos();
        double fieldRelativeCameraToTurretCenterY =
                radiusFromCameraLensToTurretCenterMeters * fieldRelativeLimelightRotation.getSin();

        Logger.recordOutput("Vision/CameraToTurretX", fieldRelativeCameraToTurretCenterX);
        Logger.recordOutput("Vision/CameraToTurretY", fieldRelativeCameraToTurretCenterY);

        double turretCenterOffsetOnRobot = Units.inchesToMeters(-4.0);

        double fieldRelativeTurretCenterToRobotCenterX =
                turretCenterOffsetOnRobot * fieldRelativeRobotOrientation.getCos();
        double fieldRelativeTurretCenterToRobotCenterY =
                turretCenterOffsetOnRobot * fieldRelativeRobotOrientation.getSin();

        Logger.recordOutput("Vision/TurretCenterToRobotCenterX", fieldRelativeTurretCenterToRobotCenterX);
        Logger.recordOutput("Vision/TurretCenterToRobotCenterY", fieldRelativeTurretCenterToRobotCenterY);

        double fieldRelativeDrivebaseCenterToAprilTagX = fieldRelativeAprilTagToCameraX
                + fieldRelativeCameraToTurretCenterX
                + fieldRelativeTurretCenterToRobotCenterX;
        double fieldRelativeDrivebaseCenterToAprilTagY = fieldRelativeAprilTagToCameraY
                + fieldRelativeCameraToTurretCenterY
                + fieldRelativeTurretCenterToRobotCenterY;

        Logger.recordOutput("Vision/DrivebaseCenterToAprilTagX", fieldRelativeDrivebaseCenterToAprilTagX);
        Logger.recordOutput("Vision/DrivebaseCenterToAprilTagY", fieldRelativeDrivebaseCenterToAprilTagY);

        double fieldRelativeRobotX = tagCoords.getX() + fieldRelativeDrivebaseCenterToAprilTagX;
        double fieldRelativeRobotY = tagCoords.getY() + fieldRelativeDrivebaseCenterToAprilTagY;

        Logger.recordOutput("Vision/FieldRelativeRobotX", fieldRelativeRobotX);
        Logger.recordOutput("Vision/FieldRelativeRobotY", fieldRelativeRobotY);

        Pose2d pose = new Pose2d(fieldRelativeRobotX, fieldRelativeRobotY, fieldRelativeRobotOrientation);
        Logger.recordOutput("Vision/RobotPose", pose);
        return Optional.of(pose);
    }

    /**
     * Returns the camera to april tag translation used for distance calculations.
     *
     * @param robotPose   The robot's pose.
     * @param turretAngle The turret's angle.
     * @param tag         The april tag's pose.
     * @return The camera to april tag translation.
     */
    public synchronized Translation2d getCurrentCameraPosition(Pose2d robotPose, Rotation2d turretAngle, Pose3d tag) {
        var robotToTurret = new Transform2d(Units.inchesToMeters(-4.0), 0, new Rotation2d());
        var turretToCamera = new Pose2d(Units.inchesToMeters(8.979), 0, new Rotation2d());
        // Calculate robot relative offset of the camera.
        var robotToCamera =
                turretToCamera.rotateBy(turretAngle).getTranslation().plus(robotToTurret.getTranslation());

        // Calculate the field relative from the robot relative position of the camera on the robot.
        var tagToCamera =
                robotPose.plus(new Transform2d(robotToCamera, new Rotation2d())).minus(tag.toPose2d());

        return tagToCamera.getTranslation();
    }

    public synchronized double getVisionHorizontalDistance() {
        if (!Double.isNaN(speakerHorizontalDistance)) {
            return speakerHorizontalDistance;
        } else {
            var distance =
                    getCurrentCameraPosition(odometryPose, getCurrentTurretAngle(), FieldConstants.getSpeakerTag());
            return distance.getNorm();
        }
    }

    public synchronized void addTurretAngleObservation(TurretAngleObservation observation) {
        turretAngleBuffer.addSample(observation.timestamp, observation.turretAngle);
    }

    /**
     * Resets the robot state.
     *
     * @param startTime The starting timestamp.
     * @param pose      The starting field-relative pose measurement.
     */
    public synchronized void reset(double startTime, Pose2d pose) {
        odometry.resetPosition(lastGyroAngle, lastWheelPositions, pose);
        odometryPose = odometry.getPoseMeters();
        poseEstimator.resetPosition(lastGyroAngle, lastWheelPositions, pose);
        poseBuffer.clear();
        poseBuffer.addSample(startTime, pose);
    }

    /**
     * Resets the robot state.
     *
     * @param pose The starting field-relative pose measurement.
     */
    public synchronized void reset(final Pose2d pose) {
        reset(Timer.getFPGATimestamp(), pose);
    }

    /**
     * Resets the robot state.
     */
    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), new Pose2d());
    }

    /**
     * Returns a predicted pose looking ahead at a time in the future using the current pose and chassis speeds.
     *
     * @param translationLookaheadTimeSeconds The translation lookahead time in seconds.
     * @param rotationLookaheadTimeSeconds    The rotation lookahead time in seconds.
     * @return The predicted pose.
     */
    public synchronized Pose2d getPredictedOdomToVehicle(
            double translationLookaheadTimeSeconds, double rotationLookaheadTimeSeconds) {
        return getPredictedOdomToVehicle(
                odometryPose, lastChassisSpeeds, translationLookaheadTimeSeconds, rotationLookaheadTimeSeconds);
    }

    /**
     * Returns a predicted pose looking ahead at a time in the future.
     *
     * @param currentPose                     The current pose.
     * @param currentSpeeds                   The current chassis speeds.
     * @param translationLookaheadTimeSeconds The translation lookahead time in seconds.
     * @param rotationLookaheadTimeSeconds    The rotation lookahead time in seconds.
     * @return The predicted pose.
     */
    public synchronized Pose2d getPredictedOdomToVehicle(
            Pose2d currentPose,
            ChassisSpeeds currentSpeeds,
            double translationLookaheadTimeSeconds,
            double rotationLookaheadTimeSeconds) {

        // Apply a twist to the pose to get the predicted pose
        return currentPose.exp(new Twist2d(
                currentSpeeds.vxMetersPerSecond * translationLookaheadTimeSeconds,
                currentSpeeds.vyMetersPerSecond * translationLookaheadTimeSeconds,
                currentSpeeds.omegaRadiansPerSecond * rotationLookaheadTimeSeconds));
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Odometry/OdometryPose")
    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    /**
     * Get the current {@link ChassisSpeeds}.
     *
     * @return the robot's current {@link ChassisSpeeds}.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return lastChassisSpeeds;
    }

    /**
     * Returns the angle from the robot's origin to the april tag location.
     *
     * @param robotPose The robot's pose.
     * @param tag       The april tag's pose.
     * @return The angle from the robot's origin to the april tag location.
     */
    @AutoLogOutput(key = "Odometry/RobotToAprilTagAngle")
    public Rotation2d getRobotToAprilTagAngle(Pose2d robotPose, Pose3d tag) {
        return Rotation2d.fromRadians(
                Angles.getLineAngle(robotPose.getTranslation(), tag.toPose2d().getTranslation()));
    }

    /**
     * Returns the robot relative angle for the turret to track an april tag.
     * <p>If a camera pose is available, it will use that to calculate the angle. Otherwise, it will use the current odometry pose.
     *
     * @param cameraPose The camera's pose.
     * @param tag        The april tag's pose.
     * @return the robot relative angle for the turret to track an april tag.
     */
    @AutoLogOutput(key = "Tracking/TurretAimingAngle")
    public synchronized Rotation2d getTurretAimingParameter(
            Pose2d cameraPose, Pose2d robotPose, Rotation2d currentTurretAngle, Pose3d tag) {
        if (cameraPose != null) {
            return getTurretAimingParameterFromCameraPose(cameraPose, currentTurretAngle);
        } else {
            return getTurretAimingParameterFromRobotPose(robotPose, tag);
        }
    }

    /**
     * Returns the robot relative angle for the turret to track an april tag.
     * <p>If a camera pose is available, it will use that to calculate the angle. Otherwise, it will use the current odometry pose.
     *
     * @param tag The april tag's pose.
     * @return the robot relative angle for the turret to track an april tag.
     */
    public synchronized Rotation2d getTurretAimingParameter(Pose3d tag) {
        return getTurretAimingParameter(getSpeakerCameraPose(), getOdometryPose(), getCurrentTurretAngle(), tag);
    }

    /**
     * Returns the robot relative angle for the turret to track an april tag using a camera pose.
     *
     * @param cameraPose         The camera's pose.
     * @param currentTurretAngle the current angle of the turret.
     * @return the robot relative angle for the turret to track an april tag.
     */
    @AutoLogOutput(key = "Tracking/TurretAimingAngleFromCameraPose")
    public Rotation2d getTurretAimingParameterFromCameraPose(Pose2d cameraPose, Rotation2d currentTurretAngle) {
        Rotation2d aprilTagAngle = cameraPose
                .getRotation()
                .times(CompensationCoefficientConstants.REGULAR_TURRET_TRACKING_CAMERA_ANGLE_COEFFICIENT);
        Rotation2d angle = currentTurretAngle.plus(aprilTagAngle);
        return angle;
    }

    /**
     * Returns the robot relative angle for the turret to track an aril tag based on the current odometry pose.
     *
     * @param robotPose The robot's pose.
     * @param tag       The april tag's pose.
     * @return the robot relative angle for the turret to track an aril tag.
     */
    @AutoLogOutput(key = "Tracking/TurretAimingAngleFromOdometryPose")
    public Rotation2d getTurretAimingParameterFromRobotPose(Pose2d robotPose, Pose3d tag) {
        var robotToTurret = new Transform2d(Units.inchesToMeters(-4.0), 0, new Rotation2d());
        var turretToTarget = robotPose.transformBy(robotToTurret);

        var robotToAprilTagAngle = getRobotToAprilTagAngle(turretToTarget, tag);
        var angle = Rotation2d.fromRadians(Angles.shortestAngularDistance(
                robotPose.getRotation().getRadians(), robotToAprilTagAngle.getRadians()));
        return angle;
    }

    public synchronized Rotation2d getCurrentTurretAngle() {
        return turretAngleBuffer.getInternalBuffer().lastEntry().getValue();
    }

    public synchronized double getDistanceToSpeakerGoal() {
        return FieldConstants.getSpeakerTag()
                .toPose2d()
                .minus(odometryPose)
                .getTranslation()
                .getNorm();
    }

    public synchronized double getDistanceToFeedTarget() {
        return FieldConstants.getRegularFeedShotTarget()
                .minus(odometryPose)
                .getTranslation()
                .getNorm();
    }

    public synchronized double getDistancToFeedFromSource() {
        return FieldConstants.getFeedShotFromSource()
                .minus(odometryPose)
                .getTranslation()
                .getNorm();
    }

    public synchronized boolean hasTarget() {
        return hasTarget;
    }

    public synchronized void setHasPiece(final boolean hasPiece) {
        Logger.recordOutput("Simulator/HasPiece", hasPiece);

        this.hasPiece = hasPiece;
    }

    public synchronized void resetPose() {
        reset(new Pose2d(
                odometryPose.getX(),
                odometryPose.getY(),
                FieldConstants.isBlueAlliance() ? new Rotation2d() : Rotation2d.fromDegrees(180)));
    }

    public synchronized void resetPoseToVisionPose() {
        if (robotPoseFromCameraPose.isPresent()) {
            reset(this.robotPoseFromCameraPose.get());
        }
    }

    public boolean withinChassisSpeeds(ChassisSpeeds threshold) {
        return Math.abs(this.lastChassisSpeeds.vxMetersPerSecond) < threshold.vxMetersPerSecond
                && Math.abs(this.lastChassisSpeeds.vyMetersPerSecond) < threshold.vyMetersPerSecond
                && Math.abs(this.lastChassisSpeeds.omegaRadiansPerSecond) < threshold.omegaRadiansPerSecond;
    }

    /**
     * Gets the aiming parameters required for shoot-on-the-fly.
     *
     * @param turretLookaheadTime The lookahead time in seconds.
     * @param pivotLookaheadTime  The lookahead time in seconds.
     * @param robotPose           The robot's pose.
     * @param tag                 The april tag's pose.
     * @param chassisSpeeds       The chassis speeds.
     * @param turretAngle         The turret's angle.
     * @return the aiming parameters required for shoot-on-the-fly.
     */
    protected synchronized AimingParameters getAimingParameters(
            double turretLookaheadTime,
            double pivotLookaheadTime,
            Pose2d robotPose,
            Pose3d tag,
            ChassisSpeeds chassisSpeeds,
            Rotation2d turretAngle) {

        // Get the predicted pose of the robot in the future.
        Pose2d lookaheadPoseForTurret =
                getPredictedOdomToVehicle(robotPose, chassisSpeeds, turretLookaheadTime, turretLookaheadTime);
        Pose2d lookaheadPoseForPivot =
                getPredictedOdomToVehicle(robotPose, chassisSpeeds, pivotLookaheadTime, pivotLookaheadTime);

        // Calculate the turret angle to track the speaker tag.
        var turretAimAngle = getTurretAimingParameterFromRobotPose(lookaheadPoseForTurret, tag);

        // Get the camera to april tag translation for the predicted robot pose.
        var cameraToAprilTag = getCurrentCameraPosition(lookaheadPoseForPivot, turretAimAngle, tag);

        // Get the distance to goal from the predicted camera position so that the
        // pivot pitch can be for the predicted robot pose.
        var distance = cameraToAprilTag.getNorm();

        // Calculate the virtual pose of the speaker tag.  This is the pose of the speaker tag
        // relative to the robot's y velocity vector.
        var virtualPose = tag.toPose2d()
                .getTranslation()
                .minus(new Translation2d(
                        0.0,
                        CompensationCoefficientConstants
                                        .SOTM_PERCENTAGE_OF_LOOKAHEAD_Y_DELTA_APPLIED_AS_VIRTUAL_GOAL_OFFSET
                                * (lookaheadPoseForTurret.getY() - robotPose.getY())));
        //                //.minus(lookaheadPose.getY()minus(robotPose.getTranslation()));

        Logger.recordOutput("AimingParameters/VirtualTargetPose", virtualPose);
        Logger.recordOutput("AimingParameters/CameraToAprilTag", cameraToAprilTag);
        Logger.recordOutput("AimingParameters/LookaheadPose", lookaheadPoseForTurret);
        Logger.recordOutput("AimingParameters/YLookaheadTime", turretLookaheadTime);
        Logger.recordOutput("AimingParameters/XLookaheadTime", pivotLookaheadTime);

        // Get the virtual 3d pose of the speaker tag.
        var virtualTagPose = new Pose3d(tag.toPose2d().getX(), virtualPose.getY(), 0.0, tag.getRotation());

        // Calculate the turret angle to track the virtual speaker tag.
        // turretAimAngle = getTurretAimingParameterFromRobotPose(lookaheadPose, virtualTagPose);

        var latestParameters = new AimingParameters(
                turretAimAngle,
                turretAimAngle.minus(turretAngle),
                cameraToAprilTag,
                chassisSpeeds.vxMetersPerSecond * virtualPose.getAngle().getSin() / distance
                        - chassisSpeeds.vyMetersPerSecond
                                * virtualPose.getAngle().getCos()
                                / distance);
        return latestParameters;
    }

    public synchronized AimingParameters getAimingParameters(double turretLookaheadTime, double pivotLookaheadTime) {
        return getAimingParameters(
                turretLookaheadTime,
                pivotLookaheadTime,
                getOdometryPose(),
                FieldConstants.getSpeakerTag(),
                getChassisSpeeds(),
                getCurrentTurretAngle());
    }

    public double getLastAccelerationVector() {
        return Math.hypot(lastAccelerationXFromPigeon, lastAccelerationYFromPigeon);
    }
}
