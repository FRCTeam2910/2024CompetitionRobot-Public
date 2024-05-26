package frc.robot.config;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

@SuppressWarnings("UnusedVariable")
public class FieldConstants {
    private FieldConstants() {}

    private static final FieldIdentity field = FieldIdentity.NONE;
    private static final FieldOffsets fieldoffsets = FieldOffsetConstants.getFieldOffsets(field);

    public static final double FIELD_LENGTH_METERS = Units.feetToMeters(54.27083) + fieldoffsets.FIELD_LENGTH_METERS;
    public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26.9375) + fieldoffsets.FIELD_WIDTH_METERS;
    public static final AprilTagFieldLayout FIELD_LAYOUT;

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isEmpty()
                || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    static {
        try {
            FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    // Field Elements
    public static final double SPEAKER_HEIGHT_METERS = Units.inchesToMeters(78);
    public static final double SPEAKER_OPENING_WIDTH_METERS = Units.inchesToMeters(41.375);
    public static final double SPEAKER_OPENING_EXTENSION_LENGTH_METERS = Units.inchesToMeters(18);
    public static final double SPEAKER_OPENING_ANGLE_RADIANS = Units.degreesToRadians(14);
    public static final double LENGTH_BETWEEN_SPEAKER_TAGS_METERS = Units.inchesToMeters(17);
    public static final double LENGTH_BETWEEN_SOURCE_TAGS_METERS = Units.inchesToMeters(38.75);
    public static final double LOWEST_POINT_OF_CHAIN_METERS = Units.inchesToMeters(28.25);

    // April Tag IDs
    public static final int BLUE_SOURCE_LEFT = 1;
    public static final int BLUE_SOURCE_RIGHT = 2;
    public static final int RED_SUBWOOFER = 3;
    public static final int RED_SPEAKER = 4;
    public static final int RED_AMP = 5;
    public static final int BLUE_AMP = 6;
    public static final int BLUE_SPEAKER = 7;
    public static final int BLUE_SUBWOOFER = 8;
    public static final int RED_SOURCE_LEFT = 9;
    public static final int RED_SOURCE_RIGHT = 10;
    /*
    Tags 11 - 16 are labelled with the alliance stage and then the direction of tag (towards source, towards amp, towards opposing alliance stage)
     */
    public static final int RED_STAGE_SOURCE = 11;
    public static final int RED_STAGE_AMP = 12;
    public static final int RED_STAGE_STAGE = 13;
    public static final int BLUE_STAGE_STAGE = 14;
    public static final int BLUE_STAGE_AMP = 15;
    public static final int BLUE_STAGE_SOURCE = 16;

    // Getters for April Tag IDs
    public static int getSourceLeftAprilTagId() {
        if (!isBlueAlliance()) {
            return RED_SOURCE_LEFT;
        }
        return BLUE_SOURCE_LEFT;
    }

    public static int getSourceRightAprilTagId() {
        if (!isBlueAlliance()) {
            return RED_SOURCE_RIGHT;
        }
        return BLUE_SOURCE_RIGHT;
    }

    public static int getSubwooferAprilTagId() {
        if (!isBlueAlliance()) {
            return RED_SUBWOOFER;
        }
        return BLUE_SUBWOOFER;
    }

    public static int getSpeakerAprilTagId() {
        if (!isBlueAlliance()) {
            return RED_SPEAKER;
        }
        return BLUE_SPEAKER;
    }

    public static int getAmpAprilTagId() {
        if (!isBlueAlliance()) {
            return RED_AMP;
        }
        return BLUE_AMP;
    }

    public static int getStageAmpAprilTagId() {
        if (!isBlueAlliance()) {
            return RED_STAGE_AMP;
        }
        return BLUE_STAGE_AMP;
    }

    public static int getStageSourceAprilTagId() {
        if (!isBlueAlliance()) {
            return RED_STAGE_SOURCE;
        }
        return BLUE_STAGE_SOURCE;
    }

    public static int getStageStageAprilTagId() {
        if (!isBlueAlliance()) {
            return RED_STAGE_STAGE;
        }
        return BLUE_STAGE_STAGE;
    }

    // Tag Coordinates
    public static Pose3d getLeftSourceTag() {
        return FIELD_LAYOUT.getTagPose(getSourceLeftAprilTagId()).get();
    }

    public static Pose3d getRightSourceTag() {
        return FIELD_LAYOUT.getTagPose(getSourceRightAprilTagId()).get();
    }

    public static Pose3d getSubwooferTag() {
        return FIELD_LAYOUT.getTagPose(getSubwooferAprilTagId()).get();
    }

    public static Pose3d getSpeakerTag() {
        Pose3d tag = FIELD_LAYOUT.getTagPose(getSpeakerAprilTagId()).get();

        return new Pose3d(tag.getX(), tag.getY(), tag.getZ(), tag.getRotation());
    }

    public static Pose2d getRegularFeedShotTarget() {
        return isBlueAlliance() ? new Pose2d(0.0, 11.0, new Rotation2d()) : new Pose2d(16.46, 12.5, new Rotation2d());
    }

    public static Pose2d getStationaryFeedShotTarget() {
        return isBlueAlliance() ? new Pose2d(0.42, 8.0, new Rotation2d()) : new Pose2d(16.0, 9.5, new Rotation2d());
    }

    public static Pose2d getFeedShotFromSource() {
        return isBlueAlliance() ? new Pose2d(7.7, 6.8, new Rotation2d()) : new Pose2d(9.11, 6.8, new Rotation2d());
    }

    public static Pose3d getAmpTag() {
        return FIELD_LAYOUT.getTagPose(getAmpAprilTagId()).get();
    }

    public static Pose3d getStageSourceTag() {
        return FIELD_LAYOUT.getTagPose(getStageSourceAprilTagId()).get();
    }

    public static Pose3d getStageAmpTag() {
        return FIELD_LAYOUT.getTagPose(getStageAmpAprilTagId()).get();
    }

    public static Pose3d getStageStageTag() {
        return FIELD_LAYOUT.getTagPose(getStageStageAprilTagId()).get();
    }

    // Game Piece Poses
    public static Pose3d getGamePiece1() {
        if (isBlueAlliance()) {
            return new Pose3d(2.896, -1.210, 0, new Rotation3d()).transformBy(fieldoffsets.BLUE_GAME_PIECE_1);
        }
        return new Pose3d(13.645, -1.210, 0, new Rotation3d()).transformBy(fieldoffsets.RED_GAME_PIECE_1);
    }

    public static Pose3d getGamePiece2() {
        if (isBlueAlliance()) {
            return new Pose3d(2.896, -2.658, 0, new Rotation3d()).transformBy(fieldoffsets.BLUE_GAME_PIECE_2);
        }
        return new Pose3d(13.645, -2.658, 0, new Rotation3d()).transformBy(fieldoffsets.RED_GAME_PIECE_2);
    }

    public static Pose3d getGamePiece3() {
        if (isBlueAlliance()) {
            return new Pose3d(2.896, -4.106, 0, new Rotation3d()).transformBy(fieldoffsets.BLUE_GAME_PIECE_3);
        }
        return new Pose3d(13.645, -4.106, 0, new Rotation3d()).transformBy(fieldoffsets.RED_GAME_PIECE_3);
    }

    public static Pose3d GAME_PIECE_4 =
            new Pose3d(8.271, -0.753, 0, new Rotation3d()).transformBy(fieldoffsets.GAME_PIECE_4);
    public static Pose3d GAME_PIECE_5 =
            new Pose3d(8.271, -2.429, 0, new Rotation3d()).transformBy(fieldoffsets.GAME_PIECE_5);
    public static Pose3d GAME_PIECE_6 =
            new Pose3d(8.271, -4.106, 0, new Rotation3d()).transformBy(fieldoffsets.GAME_PIECE_6);
    public static Pose3d GAME_PIECE_7 =
            new Pose3d(8.271, -5.782, 0, new Rotation3d()).transformBy(fieldoffsets.GAME_PIECE_7);
    public static Pose3d GAME_PIECE_8 =
            new Pose3d(8.271, -7.458, 0, new Rotation3d()).transformBy(fieldoffsets.GAME_PIECE_8);

    public static Pose3d BLUE_SPEAKER_BOTTOM_LEFT =
            new Pose3d(0.0381, 4.936617, 1.984502, new Rotation3d()).transformBy(fieldoffsets.BLUE_SPEAKER_BOTTOM_LEFT);
    public static Pose3d BLUE_SPEAKER_BOTTOM_RIGHT = new Pose3d(0.0381, 6.159119, 1.984502, new Rotation3d())
            .transformBy(fieldoffsets.BLUE_SPEAKER_BOTTOM_RIGHT);
    public static Pose3d BLUE_SPEAKER_TOP_LEFT =
            new Pose3d(0.421894, 4.936617, 2.10566, new Rotation3d()).transformBy(fieldoffsets.BLUE_SPEAKER_TOP_LEFT);
    public static Pose3d BLUE_SPEAKER_TOP_RIGHT =
            new Pose3d(0.421894, 6.159119, 2.10566, new Rotation3d()).transformBy(fieldoffsets.BLUE_SPEAKER_TOP_RIGHT);

    public static Pose3d RED_SPEAKER_BOTTOM_LEFT = new Pose3d(16.579342, 6.159119, 1.984502, new Rotation3d())
            .transformBy(fieldoffsets.RED_SPEAKER_BOTTOM_LEFT);
    public static Pose3d RED_SPEAKER_BOTTOM_RIGHT = new Pose3d(16.579342, 4.936617, 1.984502, new Rotation3d())
            .transformBy(fieldoffsets.RED_SPEAKER_BOTTOM_RIGHT);
    public static Pose3d RED_SPEAKER_TOP_LEFT =
            new Pose3d(16.119348, 6.159119, 2.10566, new Rotation3d()).transformBy(fieldoffsets.RED_SPEAKER_TOP_LEFT);
    public static Pose3d RED_SPEAKER_TOP_RIGHT =
            new Pose3d(16.119348, 4.936617, 2.10566, new Rotation3d()).transformBy(fieldoffsets.RED_SPEAKER_TOP_RIGHT);

    // TODO: Figure out coordinates or finding center based on the fact that the corners of the amp are rounded
    public static Pose3d BLUE_AMP_BOTTOM_LEFT =
            new Pose3d(1.5367, 8.2042, 0.6604, new Rotation3d()).transformBy(fieldoffsets.BLUE_AMP_BOTTOM_LEFT);
    public static Pose3d BLUE_AMP_BOTTOM_RIGHT =
            new Pose3d(2.1463, 8.2042, 0.6604, new Rotation3d()).transformBy(fieldoffsets.BLUE_AMP_BOTTOM_RIGHT);
    public static Pose3d BLUE_AMP_TOP_LEFT =
            new Pose3d(1.5367, 8.2042, 1.1176, new Rotation3d()).transformBy(fieldoffsets.BLUE_AMP_TOP_LEFT);
    public static Pose3d BLUE_AMP_TOP_RIGHT =
            new Pose3d(2.1463, 8.2042, 1.1176, new Rotation3d()).transformBy(fieldoffsets.BLUE_AMP_TOP_RIGHT);

    public static Pose3d RED_AMP_BOTTOM_LEFT =
            new Pose3d(14.395958, 8.2042, 0.6604, new Rotation3d()).transformBy(fieldoffsets.RED_AMP_BOTTOM_LEFT);
    public static Pose3d RED_AMP_BOTTOM_RIGHT =
            new Pose3d(15.005558, 8.2042, 0.6604, new Rotation3d()).transformBy(fieldoffsets.RED_AMP_BOTTOM_RIGHT);
    public static Pose3d RED_AMP_TOP_LEFT =
            new Pose3d(14.395958, 8.2042, 1.1176, new Rotation3d()).transformBy(fieldoffsets.RED_AMP_TOP_LEFT);
    public static Pose3d RED_AMP_TOP_RIGHT =
            new Pose3d(15.005558, 8.2042, 1.1176, new Rotation3d()).transformBy(fieldoffsets.RED_AMP_TOP_RIGHT);

    private static final double stageXComponent = 0.3048 * Math.cos(Math.toRadians(60));
    private static final double stageYComponent = 0.3048 * Math.sin(Math.toRadians(60));
    // ID's 14-16
    public static Pose3d BLUE_TRAP_STAGE_CENTER_BOTTOM_LEFT = new Pose3d(
                    5.320792, 4.105148 - 0.3048, 1.4351, new Rotation3d())
            .transformBy(fieldoffsets.BLUE_TRAP_STAGE_CENTER_BOTTOM_LEFT);
    public static Pose3d BLUE_TRAP_STAGE_CENTER_TOP_RIGHT = new Pose3d(
                    5.320792, 4.105148 + 0.3048, 1.8923, new Rotation3d())
            .transformBy(fieldoffsets.BLUE_TRAP_STAGE_CENTER_TOP_RIGHT);
    public static Pose3d BLUE_TRAP_STAGE_LEFT_BOTTOM_LEFT = new Pose3d(
                    4.641342 + stageXComponent, 3.713226 + stageYComponent, 1.4351, new Rotation3d())
            .transformBy(fieldoffsets.BLUE_TRAP_STAGE_LEFT_BOTTOM_LEFT);
    public static Pose3d BLUE_TRAP_STAGE_LEFT_TOP_RIGHT = new Pose3d(
                    4.641342 - stageXComponent, 3.713226 - stageYComponent, 1.8923, new Rotation3d())
            .transformBy(fieldoffsets.BLUE_TRAP_STAGE_LEFT_TOP_RIGHT);
    public static Pose3d BLUE_TRAP_STAGE_RIGHT_BOTTOM_LEFT = new Pose3d(
                    4.641342 - stageXComponent, 3.713226 + stageYComponent, 1.4351, new Rotation3d())
            .transformBy(fieldoffsets.BLUE_TRAP_STAGE_RIGHT_BOTTOM_LEFT);
    public static Pose3d BLUE_TRAP_STAGE_RIGHT_TOP_RIGHT = new Pose3d(
                    4.641342 + stageXComponent, 3.713226 - stageYComponent, 1.8923, new Rotation3d())
            .transformBy(fieldoffsets.BLUE_TRAP_STAGE_RIGHT_TOP_RIGHT);

    // ID's 11-13
    public static Pose3d RED_TRAP_STAGE_LEFT_BOTTOM_LEFT = new Pose3d(
                    11.904726 - stageXComponent, 3.713226 - stageYComponent, 1.4351, new Rotation3d())
            .transformBy(fieldoffsets.RED_TRAP_STAGE_LEFT_BOTTOM_LEFT);
    public static Pose3d RED_TRAP_STAGE_LEFT_TOP_RIGHT = new Pose3d(
                    11.904726 + stageXComponent, 3.713226 + stageYComponent, 1.8923, new Rotation3d())
            .transformBy(fieldoffsets.RED_TRAP_STAGE_LEFT_TOP_RIGHT);
    public static Pose3d RED_TRAP_STAGE_RIGHT_BOTTOM_LEFT = new Pose3d(
                    11.904726 + stageXComponent, 4.49834 - stageYComponent, 1.4351, new Rotation3d())
            .transformBy(fieldoffsets.RED_TRAP_STAGE_RIGHT_BOTTOM_LEFT);
    public static Pose3d RED_TRAP_STAGE_RIGHT_TOP_RIGHT = new Pose3d(
                    11.904726 - stageXComponent, 4.49834 + stageYComponent, 1.8923, new Rotation3d())
            .transformBy(fieldoffsets.RED_TRAP_STAGE_RIGHT_TOP_RIGHT);
    public static Pose3d RED_TRAP_STAGE_CENTER_BOTTOM_LEFT = new Pose3d(
                    11.220196, 4.105148 + 0.3048, 1.4351, new Rotation3d())
            .transformBy(fieldoffsets.RED_TRAP_STAGE_CENTER_BOTTOM_LEFT);
    public static Pose3d RED_TRAP_STAGE_CENTER_TOP_RIGHT = new Pose3d(
                    11.220196, 4.105148 - 0.3048, 1.8923, new Rotation3d())
            .transformBy(fieldoffsets.RED_TRAP_STAGE_CENTER_TOP_RIGHT);

    public static Pose3d getSpeakerCenter() {
        if (isBlueAlliance()) {
            var midpointX = (BLUE_SPEAKER_TOP_RIGHT.getX() + BLUE_SPEAKER_BOTTOM_LEFT.getX()) / 2;
            var midpointY = (BLUE_SPEAKER_TOP_RIGHT.getY() + BLUE_SPEAKER_BOTTOM_LEFT.getY()) / 2;
            var midpointZ = (BLUE_SPEAKER_TOP_RIGHT.getZ() + BLUE_SPEAKER_BOTTOM_LEFT.getZ()) / 2;
            return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d());
        }
        var midpointX = (RED_SPEAKER_TOP_RIGHT.getX() + RED_SPEAKER_BOTTOM_LEFT.getX()) / 2;
        var midpointY = (RED_SPEAKER_TOP_RIGHT.getY() + RED_SPEAKER_BOTTOM_LEFT.getY()) / 2;
        var midpointZ = (RED_SPEAKER_TOP_RIGHT.getZ() + RED_SPEAKER_BOTTOM_LEFT.getZ()) / 2;
        return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d());
    }

    public static Pose3d getAmpCenter() {
        if (isBlueAlliance()) {
            var midpointX = (BLUE_AMP_TOP_RIGHT.getX() + BLUE_AMP_BOTTOM_LEFT.getX()) / 2;
            var midpointY = (BLUE_AMP_TOP_RIGHT.getY() + BLUE_AMP_BOTTOM_LEFT.getY()) / 2;
            var midpointZ = (BLUE_AMP_TOP_RIGHT.getZ() + BLUE_AMP_BOTTOM_LEFT.getZ()) / 2;
            return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d(0, 0, 270));
        }
        var midpointX = (RED_AMP_TOP_RIGHT.getX() + RED_AMP_BOTTOM_LEFT.getX()) / 2;
        var midpointY = (RED_AMP_TOP_RIGHT.getY() + RED_AMP_BOTTOM_LEFT.getY()) / 2;
        var midpointZ = (RED_AMP_TOP_RIGHT.getZ() + RED_AMP_BOTTOM_LEFT.getZ()) / 2;
        return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d(0, 0, 270));
    }

    public static Pose3d getCenterStageTrapCenter() {
        if (isBlueAlliance()) {
            var midpointX = (BLUE_TRAP_STAGE_CENTER_TOP_RIGHT.getX() + BLUE_TRAP_STAGE_CENTER_BOTTOM_LEFT.getX()) / 2;
            var midpointY = (BLUE_TRAP_STAGE_CENTER_TOP_RIGHT.getY() + BLUE_TRAP_STAGE_CENTER_BOTTOM_LEFT.getY()) / 2;
            var midpointZ = (BLUE_TRAP_STAGE_CENTER_TOP_RIGHT.getZ() + BLUE_TRAP_STAGE_CENTER_BOTTOM_LEFT.getZ()) / 2;
            return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d(0, 0, 0));
        }
        var midpointX = (RED_TRAP_STAGE_CENTER_TOP_RIGHT.getX() + RED_TRAP_STAGE_CENTER_BOTTOM_LEFT.getX()) / 2;
        var midpointY = (RED_TRAP_STAGE_CENTER_TOP_RIGHT.getY() + RED_TRAP_STAGE_CENTER_BOTTOM_LEFT.getY()) / 2;
        var midpointZ = (RED_TRAP_STAGE_CENTER_TOP_RIGHT.getZ() + RED_TRAP_STAGE_CENTER_BOTTOM_LEFT.getZ()) / 2;
        return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d(0, 0, 180));
    }

    public static Pose3d getLeftStageTrapCenter() {
        if (isBlueAlliance()) {
            var midpointX = (BLUE_TRAP_STAGE_LEFT_TOP_RIGHT.getX() + BLUE_TRAP_STAGE_LEFT_BOTTOM_LEFT.getX()) / 2;
            var midpointY = (BLUE_TRAP_STAGE_LEFT_TOP_RIGHT.getY() + BLUE_TRAP_STAGE_LEFT_BOTTOM_LEFT.getY()) / 2;
            var midpointZ = (BLUE_TRAP_STAGE_LEFT_TOP_RIGHT.getZ() + BLUE_TRAP_STAGE_LEFT_BOTTOM_LEFT.getZ()) / 2;
            return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d(0, 0, 120));
        }
        var midpointX = (RED_TRAP_STAGE_LEFT_TOP_RIGHT.getX() + RED_TRAP_STAGE_LEFT_BOTTOM_LEFT.getX()) / 2;
        var midpointY = (RED_TRAP_STAGE_LEFT_TOP_RIGHT.getY() + RED_TRAP_STAGE_LEFT_BOTTOM_LEFT.getY()) / 2;
        var midpointZ = (RED_TRAP_STAGE_LEFT_TOP_RIGHT.getZ() + RED_TRAP_STAGE_LEFT_BOTTOM_LEFT.getZ()) / 2;
        return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d(0, 0, 300));
    }

    public static Pose3d getRightStageTrapCenter() {
        if (isBlueAlliance()) {
            var midpointX = (BLUE_TRAP_STAGE_RIGHT_TOP_RIGHT.getX() + BLUE_TRAP_STAGE_RIGHT_BOTTOM_LEFT.getX()) / 2;
            var midpointY = (BLUE_TRAP_STAGE_RIGHT_TOP_RIGHT.getY() + BLUE_TRAP_STAGE_RIGHT_BOTTOM_LEFT.getY()) / 2;
            var midpointZ = (BLUE_TRAP_STAGE_RIGHT_TOP_RIGHT.getZ() + BLUE_TRAP_STAGE_RIGHT_BOTTOM_LEFT.getZ()) / 2;
            return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d(0, 0, 240));
        }
        var midpointX = (RED_TRAP_STAGE_RIGHT_TOP_RIGHT.getX() + RED_TRAP_STAGE_RIGHT_BOTTOM_LEFT.getX()) / 2;
        var midpointY = (RED_TRAP_STAGE_RIGHT_TOP_RIGHT.getY() + RED_TRAP_STAGE_RIGHT_BOTTOM_LEFT.getY()) / 2;
        var midpointZ = (RED_TRAP_STAGE_RIGHT_TOP_RIGHT.getZ() + RED_TRAP_STAGE_RIGHT_BOTTOM_LEFT.getZ()) / 2;
        return new Pose3d(midpointX, midpointY, midpointZ, new Rotation3d(0, 0, 60));
    }
}
