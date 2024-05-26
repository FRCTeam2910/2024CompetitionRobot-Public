package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    default void updateInputs(VisionIOInputs inputs) {}

    default void setLeds(boolean on) {}

    @AutoLog
    class VisionIOInputs {
        /**
         * Last timestamp of photo taken from LL
         */
        public double lastTimeStamp = 0.0;
        /**
         * True if limelight sees target in its frame
         */
        public boolean hasTargets = false;
        /**
         * Vertical angle from center of lens to target.
         */
        public double verticalAngleRadians = 0.0;
        /**
         * Horizontal angle from center of lens to target.
         */
        public double horizontalAngleRadians = 0.0;

        /**
         * Botpose taken from LL.
         */
        public double[] botpose = new double[6];

        /**
         * TagID of target fiducial marker.
         */
        public int tagId = -1;
    }
}
