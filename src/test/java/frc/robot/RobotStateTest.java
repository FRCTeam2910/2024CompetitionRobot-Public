package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.config.FieldConstants;
import frc.robot.util.Angles;
import org.junit.jupiter.api.Test;

import static frc.robot.config.FieldConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

@SuppressWarnings("UnusedVariable")
class RobotStateTest {
    public static final double EPSILON = 1e-6;

    @Test
    void getRobotToAprilTagAngle() {
        // Robot midfield pointing towards RED alliance station. Y is aligned Y with speaker tag location.
        Pose2d robotPose = new Pose2d(
                FieldConstants.FIELD_LENGTH_METERS / 2.0,
                FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get().getY(),
                Rotation2d.fromDegrees(0));

        var aprilTagPoseBlue = FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get();
        var aprilTagPoseRed = FIELD_LAYOUT.getTagPose(RED_SPEAKER).get();

        var angle = RobotState.getInstance().getRobotToAprilTagAngle(robotPose, aprilTagPoseBlue);
        assertEquals(180.0, angle.getDegrees(), EPSILON);

        // Note - Red and Blue speaker tags have same Y value so the angle should be 180 when
        // robot is pointing to Blue alliance wall
        angle = RobotState.getInstance().getRobotToAprilTagAngle(robotPose, aprilTagPoseRed);
        assertEquals(0.0, angle.getDegrees(), EPSILON);
    }

    @Test
    void getTurretAngleToTrackAprilTag() {
        // Robot midfield pointing towards RED alliance station. Y is aligned Y with speaker tag location.
        Pose2d robotPose = new Pose2d(
                FieldConstants.FIELD_LENGTH_METERS / 2.0,
                FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get().getY(),
                Rotation2d.fromDegrees(0));

        var aprilTagPoseBlue = FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get();
        var aprilTagPoseRed = FIELD_LAYOUT.getTagPose(RED_SPEAKER).get();

        var angle = RobotState.getInstance().getTurretAimingParameterFromRobotPose(robotPose, aprilTagPoseBlue);
        assertEquals(180.0, Math.abs(angle.getDegrees()), EPSILON);

        angle = RobotState.getInstance().getTurretAimingParameterFromRobotPose(robotPose, aprilTagPoseRed);
        assertEquals(0.0, angle.getDegrees(), EPSILON);

        // Robot midfield pointing towards to the left. Y is aligned Y with speaker tag location.
        robotPose = new Pose2d(robotPose.getTranslation(), Rotation2d.fromDegrees(90));
        angle = RobotState.getInstance().getTurretAimingParameterFromRobotPose(robotPose, aprilTagPoseBlue);
        assertTrue(90.0 > angle.getDegrees());

        angle = RobotState.getInstance().getTurretAimingParameterFromRobotPose(robotPose, aprilTagPoseRed);
        assertTrue(-90.0 < angle.getDegrees());

        // Robot midfield pointing towards BLUE alliance station. Y is aligned Y with speaker tag location.
        robotPose = new Pose2d(robotPose.getTranslation(), Rotation2d.fromDegrees(180));
        angle = RobotState.getInstance().getTurretAimingParameterFromRobotPose(robotPose, aprilTagPoseBlue);
        assertEquals(0.0, angle.getDegrees(), EPSILON);

        angle = RobotState.getInstance().getTurretAimingParameterFromRobotPose(robotPose, aprilTagPoseRed);
        assertEquals(180.0, Math.abs(angle.getDegrees()), EPSILON);

        // Robot midfield pointing towards to the right. Y is aligned Y with speaker tag location.
        robotPose = new Pose2d(robotPose.getTranslation(), Rotation2d.fromDegrees(-90));
        angle = RobotState.getInstance().getTurretAimingParameterFromRobotPose(robotPose, aprilTagPoseBlue);
        assertTrue(-90.0 < angle.getDegrees());

        angle = RobotState.getInstance().getTurretAimingParameterFromRobotPose(robotPose, aprilTagPoseRed);
        assertTrue(90.0 > angle.getDegrees());
    }

    @Test
    void getPredictedOdomToVehicle() {
        var pose = new Pose2d();
        var chassisSpeeds = new ChassisSpeeds(1, 0, 0);
        var lookaheadTimeSeconds = 1.0;

        var newPose = RobotState.getInstance()
                .getPredictedOdomToVehicle(pose, chassisSpeeds, lookaheadTimeSeconds, lookaheadTimeSeconds);
        assertEquals(new Pose2d(1, 0, new Rotation2d()), newPose);

        chassisSpeeds = new ChassisSpeeds(0, 1, 0);
        newPose = RobotState.getInstance()
                .getPredictedOdomToVehicle(newPose, chassisSpeeds, lookaheadTimeSeconds, lookaheadTimeSeconds);
        assertEquals(new Pose2d(1, 1, new Rotation2d()), newPose);

        chassisSpeeds = new ChassisSpeeds(0, 0, Units.degreesToRadians(90));
        newPose = RobotState.getInstance()
                .getPredictedOdomToVehicle(newPose, chassisSpeeds, lookaheadTimeSeconds, lookaheadTimeSeconds);
        assertEquals(new Pose2d(1, 1, Rotation2d.fromDegrees(90)), newPose);
    }

    @Test
    void latencyCompensatedVisionTest() {
        var cameraPose = new Pose2d(5.0 - Units.inchesToMeters(8.979 - 4.0), 3.0, Rotation2d.fromDegrees(180));
        var previousTurretAngle = Rotation2d.fromDegrees(0);
        var currentTurretAngle = Rotation2d.fromDegrees(0);

        var previousRobotPose = new Pose2d();
        var currentRobotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        var pose = RobotState.getInstance()
                .latencyCompensateVision(
                        cameraPose, previousRobotPose, currentRobotPose, previousTurretAngle, currentTurretAngle);
        assertEquals(cameraPose.getX(), pose.getX(), EPSILON);
        assertEquals(cameraPose.getY(), pose.getY(), EPSILON);
        assertEquals(cameraPose.getRotation().getDegrees(), pose.getRotation().getDegrees(), EPSILON);

        currentRobotPose = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
        pose = RobotState.getInstance()
                .latencyCompensateVision(
                        cameraPose, previousRobotPose, currentRobotPose, previousTurretAngle, currentTurretAngle);
        assertEquals(cameraPose.getX() + 1, pose.getX(), EPSILON);
        assertEquals(cameraPose.getY(), pose.getY(), EPSILON);
        assertEquals(cameraPose.getRotation().getDegrees(), pose.getRotation().getDegrees(), EPSILON);

        currentRobotPose = new Pose2d(0, 1, Rotation2d.fromDegrees(0));
        pose = RobotState.getInstance()
                .latencyCompensateVision(
                        cameraPose, previousRobotPose, currentRobotPose, previousTurretAngle, currentTurretAngle);
        assertEquals(cameraPose.getX(), pose.getX(), EPSILON);
        assertEquals(cameraPose.getY() + 1, pose.getY(), EPSILON);
        assertEquals(cameraPose.getRotation().getDegrees(), pose.getRotation().getDegrees(), EPSILON);

        currentRobotPose = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
        pose = RobotState.getInstance()
                .latencyCompensateVision(
                        cameraPose, previousRobotPose, currentRobotPose, previousTurretAngle, currentTurretAngle);
        assertEquals(cameraPose.getX() + 1, pose.getX(), EPSILON);
        assertEquals(cameraPose.getY() + 1, pose.getY(), EPSILON);
        assertEquals(cameraPose.getRotation().getDegrees(), pose.getRotation().getDegrees(), EPSILON);

        currentRobotPose = new Pose2d(1, 0, Rotation2d.fromDegrees(90));
        pose = RobotState.getInstance()
                .latencyCompensateVision(
                        cameraPose, previousRobotPose, currentRobotPose, previousTurretAngle, currentTurretAngle);
        assertEquals(cameraPose.getX() + 1, pose.getX(), EPSILON);
        assertEquals(cameraPose.getY(), pose.getY(), EPSILON);
        assertEquals(cameraPose.getRotation().getDegrees(), pose.getRotation().getDegrees(), EPSILON);

        currentRobotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        currentTurretAngle = Rotation2d.fromDegrees(10);
        pose = RobotState.getInstance()
                .latencyCompensateVision(
                        cameraPose, previousRobotPose, currentRobotPose, previousTurretAngle, currentTurretAngle);
        assertEquals(cameraPose.getX(), pose.getX(), EPSILON);
        assertEquals(cameraPose.getY(), pose.getY(), EPSILON);
        assertEquals(-170.0, pose.getRotation().getDegrees(), EPSILON);

        currentTurretAngle = Rotation2d.fromDegrees(-10);
        pose = RobotState.getInstance()
                .latencyCompensateVision(
                        cameraPose, previousRobotPose, currentRobotPose, previousTurretAngle, currentTurretAngle);
        assertEquals(cameraPose.getX(), pose.getX(), EPSILON);
        assertEquals(cameraPose.getY(), pose.getY(), EPSILON);
        assertEquals(170.0, pose.getRotation().getDegrees(), EPSILON);
    }

    @Test
    void calculateRobotPoseFromCameraPose() {
        var aprilTagPoseBlue = FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get();
        Pose2d cameraToTag = new Pose2d(
                //                FieldConstants.FIELD_LENGTH_METERS / 2.0,
                2.0, FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get().getY() + 2.0, Rotation2d.fromDegrees(0));

        var turretAngle = Rotation2d.fromDegrees(-90);

        var turretToCamera = new Pose2d(new Translation2d(Units.inchesToMeters(8.979), 0), new Rotation2d(0, 0));

        var robotToCamera = turretToCamera.rotateBy(turretAngle);
        var robotPose = cameraToTag.plus(new Transform2d(robotToCamera.getTranslation(), robotToCamera.getRotation()));

        var cameraToTargetAngle = Rotation2d.fromRadians(Angles.getLineAngle(
                        robotPose.getTranslation(), aprilTagPoseBlue.toPose2d().getTranslation()))
                .plus(cameraToTag.getRotation());

        assertTrue(true);
    }

    @Test
    void getRobotToField() {
        var aprilTagPose = FIELD_LAYOUT.getTagPose(RED_SPEAKER).get();

        // Midfield, facing Red Alliance Station, Y aligned with speaker tag
        var turretAngle = Rotation2d.fromDegrees(0);
        var cameraAngle = new Rotation2d();
        var robotOrientation = new Rotation2d();
        // distance from camera to tag
        var distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;

        // Vision Pose is distance away from the tag => a transform
        var visionPose = new Pose2d(
                new Translation2d(
                        distance * Math.cos(cameraAngle.getRadians()), distance * Math.sin(cameraAngle.getRadians())),
                cameraAngle);

        var pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);
        var pose2 = RobotState.getInstance()
                .getRobotPoseFromCameraPose(Optional.of(visionPose), new Pose2d(), turretAngle, aprilTagPose, distance);

        assertTrue(true);

        aprilTagPose = FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get();

        // Midfield, facing Red Alliance Station, Y aligned with speaker tag
        turretAngle = Rotation2d.fromDegrees(180);

        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);
        pose2 = RobotState.getInstance()
                .getRobotPoseFromCameraPose(Optional.of(visionPose), new Pose2d(), turretAngle, aprilTagPose, distance);

        //  Facing to the left, Y aligned with speaker tag
        turretAngle = Rotation2d.fromDegrees(90);
        robotOrientation = Rotation2d.fromDegrees(90);
        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);
        pose2 = RobotState.getInstance()
                .getRobotPoseFromCameraPose(
                        Optional.of(visionPose),
                        new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)),
                        turretAngle,
                        aprilTagPose,
                        distance);

        //  Facing to the right, Y aligned with speaker tag
        turretAngle = Rotation2d.fromDegrees(-90);
        robotOrientation = Rotation2d.fromDegrees(-90);

        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);
        pose2 = RobotState.getInstance()
                .getRobotPoseFromCameraPose(
                        Optional.of(visionPose),
                        new Pose2d(new Translation2d(), Rotation2d.fromDegrees(-90)),
                        turretAngle,
                        aprilTagPose,
                        distance);

        assertTrue(true);

        // left side facing left,
        turretAngle = Rotation2d.fromDegrees(135);

        distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;
        cameraAngle = Rotation2d.fromDegrees(0);
        robotOrientation = Rotation2d.fromDegrees(90);

        // Vision Pose is distance away from the tag => a transform
        visionPose = new Pose2d(
                new Translation2d(
                        distance * Math.cos(cameraAngle.getRadians()), distance * Math.sin(cameraAngle.getRadians())),
                cameraAngle);

        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);

        assertTrue(true);

        // right side, facing  side facing left,
        turretAngle = Rotation2d.fromDegrees(180);

        distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;
        cameraAngle = Rotation2d.fromDegrees(0);
        robotOrientation = Rotation2d.fromDegrees(-45);

        // Vision Pose is distance away from the tag => a transform
        visionPose = new Pose2d(
                new Translation2d(
                        distance * Math.cos(cameraAngle.getRadians()), distance * Math.sin(cameraAngle.getRadians())),
                cameraAngle);

        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);

        assertTrue(true);

        //        var pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle,
        // robotOrientation);
        //        var pose2 = RobotState.getInstance()
        //                .getRobotPoseFromCameraPose(Optional.of(visionPose), new Pose2d(), turretAngle, aprilTagPose,
        // distance);
        //
        //        assertTrue(true);
        //
        //        aprilTagPose = FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get();
        //
        //        // Midfield, facing Red Alliance Station, Y aligned with speaker tag
        //        turretAngle = Rotation2d.fromDegrees(180);
        //
        //        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle,
        // robotOrientation);
        //        pose2 = RobotState.getInstance()
        //                .getRobotPoseFromCameraPose(Optional.of(visionPose), new Pose2d(), turretAngle, aprilTagPose,
        // distance);
        //
        //        //  Facing to the left, Y aligned with speaker tag
        //        turretAngle = Rotation2d.fromDegrees(90);
        //        robotOrientation = Rotation2d.fromDegrees(90);
        //        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle,
        // robotOrientation);
        //        pose2 = RobotState.getInstance()
        //                .getRobotPoseFromCameraPose(
        //                        Optional.of(visionPose),
        //                        new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)),
        //                        turretAngle,
        //                        aprilTagPose,
        //                        distance);
        //
        //        //  Facing to the right, Y aligned with speaker tag
        //        turretAngle = Rotation2d.fromDegrees(-90);
        //        robotOrientation = Rotation2d.fromDegrees(-90);
        //
        //        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle,
        // robotOrientation);
        //        pose2 = RobotState.getInstance()
        //                .getRobotPoseFromCameraPose(
        //                        Optional.of(visionPose),
        //                        new Pose2d(new Translation2d(), Rotation2d.fromDegrees(-90)),
        //                        turretAngle,
        //                        aprilTagPose,
        //                        distance);
        //
        //        assertTrue(true);
        //
        //        // left side facing left,
        //        turretAngle = Rotation2d.fromDegrees(135);
        //
        //        distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;
        //        cameraAngle = Rotation2d.fromDegrees(0);
        //        robotOrientation = Rotation2d.fromDegrees(90);
        //
        //        // Vision Pose is distance away from the tag => a transform
        //        visionPose = new Pose2d(
        //                new Translation2d(
        //                        distance * Math.cos(cameraAngle.getRadians()), distance *
        // Math.sin(cameraAngle.getRadians())),
        //                cameraAngle);
        //
        //        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle,
        // robotOrientation);
        //
        //        assertTrue(true);
        //
        //        // right side, facing  side facing left,
        //        turretAngle = Rotation2d.fromDegrees(180);
        //
        //        distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;
        //        cameraAngle = Rotation2d.fromDegrees(0);
        //        robotOrientation = Rotation2d.fromDegrees(-45);
        //
        //        // Vision Pose is distance away from the tag => a transform
        //        visionPose = new Pose2d(
        //                new Translation2d(
        //                        distance * Math.cos(cameraAngle.getRadians()), distance *
        // Math.sin(cameraAngle.getRadians())),
        //                cameraAngle);
        //
        //        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle,
        // robotOrientation);
        //
        //        assertTrue(true);

        // left side, robot facing leftish, looking at RED Speaker
        aprilTagPose = FIELD_LAYOUT.getTagPose(RED_SPEAKER).get();
        turretAngle = Rotation2d.fromDegrees(-110);

        distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;
        cameraAngle = Rotation2d.fromDegrees(0);
        robotOrientation = Rotation2d.fromDegrees(70);

        // Vision Pose is distance away from the tag => a transform
        visionPose = new Pose2d(
                distance * Math.cos(cameraAngle.getRadians()),
                distance * Math.sin(cameraAngle.getRadians()),
                cameraAngle);

        pose = RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);

        assertTrue(true);
    }

    @Test
    void getCurrentCameraPositionRedSpeaker1() {
        var aprilTagPose = FIELD_LAYOUT.getTagPose(RED_SPEAKER).get();

        // Midfield, facing Red Alliance Station, Y aligned with speaker tag
        var turretAngle = Rotation2d.fromDegrees(0);
        var cameraAngle = new Rotation2d();
        var robotOrientation = new Rotation2d();
        // distance from camera to tag
        var distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;

        // Vision Pose is distance away from the tag => a,  = new Pose2d(
        var visionPose = new Pose2d(
                distance * Math.cos(cameraAngle.getRadians()),
                distance * Math.sin(cameraAngle.getRadians()),
                cameraAngle);

        Optional<Pose2d> resultRobotPose =
                RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);
        var cameraEstimatedPosition =
                RobotState.getInstance().getCurrentCameraPosition(resultRobotPose.get(), turretAngle, aprilTagPose);

        assertEquals(distance, cameraEstimatedPosition.getNorm(), EPSILON);
    }

    @Test
    void getCurrentCameraPositionBlueSpeaker5() {
        // Test case # 5
        var aprilTagPose = FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get();

        // left side facing left,
        var turretAngle = Rotation2d.fromDegrees(135);

        var distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;
        var cameraAngle = Rotation2d.fromDegrees(0);
        var robotOrientation = Rotation2d.fromDegrees(90);

        // Vision Pose is distance away from the tag => a transform
        var visionPose = new Pose2d(
                new Translation2d(
                        distance * Math.cos(cameraAngle.getRadians()), distance * Math.sin(cameraAngle.getRadians())),
                cameraAngle);

        Optional<Pose2d> resultRobotPose =
                RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);
        var cameraEstimatedPosition =
                RobotState.getInstance().getCurrentCameraPosition(resultRobotPose.get(), turretAngle, aprilTagPose);

        assertEquals(distance, cameraEstimatedPosition.getNorm(), EPSILON);
    }

    @Test
    void getCurrentCameraPositionBlueSpeaker6() {
        // Test case # 5
        var aprilTagPose = FIELD_LAYOUT.getTagPose(BLUE_SPEAKER).get();

        // left side facing left,
        var turretAngle = Rotation2d.fromDegrees(180);

        var distance = FieldConstants.FIELD_LENGTH_METERS / 2.0;
        var cameraAngle = Rotation2d.fromDegrees(0);
        var robotOrientation = Rotation2d.fromDegrees(-45);

        // Vision Pose is distance away from the tag => a transform
        var visionPose = new Pose2d(
                new Translation2d(
                        distance * Math.cos(cameraAngle.getRadians()), distance * Math.sin(cameraAngle.getRadians())),
                cameraAngle);

        Optional<Pose2d> resultRobotPose =
                RobotState.getInstance().getRobotToField(visionPose, aprilTagPose, turretAngle, robotOrientation);
        var cameraEstimatedPosition =
                RobotState.getInstance().getCurrentCameraPosition(resultRobotPose.get(), turretAngle, aprilTagPose);

        assertEquals(distance, cameraEstimatedPosition.getNorm(), EPSILON);
    }
}
