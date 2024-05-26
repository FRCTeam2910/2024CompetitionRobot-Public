package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldOffsetConstants {
    private FieldOffsetConstants() {}

    public static final FieldOffsets practiceOffsets = new FieldOffsets();

    static {
        practiceOffsets.FIELD_LENGTH_METERS = 0.0;
        practiceOffsets.FIELD_WIDTH_METERS = 0.0;
        practiceOffsets.APRIL_TAG_1 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_2 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_3 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_4 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_5 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_6 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_7 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_8 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_9 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_10 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_11 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_12 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_13 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_14 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_15 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.APRIL_TAG_16 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_GAME_PIECE_1 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_GAME_PIECE_2 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_GAME_PIECE_3 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_GAME_PIECE_1 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_GAME_PIECE_2 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_GAME_PIECE_3 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.GAME_PIECE_4 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.GAME_PIECE_5 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.GAME_PIECE_6 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.GAME_PIECE_7 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.GAME_PIECE_8 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.GAME_PIECE_9 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.GAME_PIECE_10 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.GAME_PIECE_11 = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_SPEAKER_BOTTOM_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_SPEAKER_BOTTOM_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_SPEAKER_TOP_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_SPEAKER_TOP_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_SPEAKER_BOTTOM_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_SPEAKER_BOTTOM_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_SPEAKER_TOP_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_SPEAKER_TOP_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_AMP_BOTTOM_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_AMP_BOTTOM_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_AMP_TOP_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.BLUE_AMP_TOP_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_AMP_BOTTOM_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        practiceOffsets.RED_AMP_BOTTOM_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
    }

    public static final FieldOffsets bonneyLakeOffsets = new FieldOffsets();

    static {
        bonneyLakeOffsets.FIELD_LENGTH_METERS = 0.0;
        bonneyLakeOffsets.FIELD_WIDTH_METERS = 0.0;
        bonneyLakeOffsets.APRIL_TAG_1 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_2 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_3 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_4 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_5 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_6 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_7 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_8 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_9 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_10 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_11 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_12 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_13 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_14 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_15 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.APRIL_TAG_16 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_GAME_PIECE_1 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_GAME_PIECE_2 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_GAME_PIECE_3 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_GAME_PIECE_1 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_GAME_PIECE_2 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_GAME_PIECE_3 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.GAME_PIECE_4 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.GAME_PIECE_5 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.GAME_PIECE_6 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.GAME_PIECE_7 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.GAME_PIECE_8 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.GAME_PIECE_9 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.GAME_PIECE_10 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.GAME_PIECE_11 = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_SPEAKER_BOTTOM_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_SPEAKER_BOTTOM_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_SPEAKER_TOP_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_SPEAKER_TOP_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_SPEAKER_BOTTOM_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_SPEAKER_BOTTOM_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_SPEAKER_TOP_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_SPEAKER_TOP_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_AMP_BOTTOM_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_AMP_BOTTOM_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_AMP_TOP_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.BLUE_AMP_TOP_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_AMP_BOTTOM_LEFT = new Transform3d(new Translation3d(), new Rotation3d());
        bonneyLakeOffsets.RED_AMP_BOTTOM_RIGHT = new Transform3d(new Translation3d(), new Rotation3d());
    }

    public static FieldOffsets getFieldOffsets(FieldIdentity field) {
        switch (field) {
            case PRACTICE -> {
                return practiceOffsets;
            }
            case BONNEY_LAKE -> {
                return bonneyLakeOffsets;
            }
            default -> {
                return new FieldOffsets();
            }
        }
    }
}
