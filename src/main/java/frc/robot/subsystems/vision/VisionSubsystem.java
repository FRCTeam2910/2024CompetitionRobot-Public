package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.config.FieldConstants;
import frc.robot.config.LimelightConfiguration;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    /**
     * Position of camera on the robot.
     * This is calculated through the horizontal distance (in meters) from the fiducial to the lens of the camera.
     */
    private Pose2d cameraToAprilTagPose = new Pose2d();

    /**
     * Horizontal Distance (in meters) from the Fiducial marker to the Lens of the camera.
     */
    private double horizontalDistanceToTargetMeters = 0;

    /**
     * Contains information about the mounting of the limelight on the robot.
     * This is required when calculating the horizontalDistanceToTargetMeters.
     */
    private final LimelightConfiguration limelightConfiguration;

    public VisionSubsystem(VisionIO visionIO, LimelightConfiguration limelightConfiguration) {
        // Initializing Fields
        this.visionIO = visionIO;

        this.limelightConfiguration = limelightConfiguration;
    }

    @Override
    public void periodic() {
        // Updates limelight inputs
        visionIO.updateInputs(inputs);
        Logger.processInputs(limelightConfiguration.Name, inputs);

        cameraToAprilTagPose = null;
        horizontalDistanceToTargetMeters = Double.NaN;

        if (hasTargets()) {
            // Calculate total pitch of camera. It is a combination of the ty value and the mounting angle of the LL.
            double theta = getPitchRadians() + limelightConfiguration.LimelightMountingPitchRadians;
            var tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(inputs.tagId);

            if (tagPose.isPresent()) {
                var pose = tagPose.get();
                // Calculate height from camera to target by subtracting the height of the April Tag and the height of
                // the LL on the robot.
                double heightFromRobotToTargetMeters = pose.getZ() - limelightConfiguration.LimelightHeightOffsetMeters;

                // Finds horizontal distance from camera to target.
                horizontalDistanceToTargetMeters = heightFromRobotToTargetMeters / Math.tan(theta);

                // Get Camera to Tag pose
                cameraToAprilTagPose = new Pose2d(
                        getHorizontalDistanceToTargetMeters() * Math.cos(getYawRadians()),
                        getHorizontalDistanceToTargetMeters() * Math.sin(getYawRadians()),
                        Rotation2d.fromRadians(getYawRadians()));

                Logger.recordOutput("Vision/CameraToAprilTag" + inputs.tagId, cameraToAprilTagPose);
            }
        }

        // Pushing Camera to tag pose to RobotState class.
        RobotState.getInstance()
                .addVisionFromAprilTagObservation(new RobotState.VisionFromAprilTagObservation(
                        inputs.lastTimeStamp,
                        cameraToAprilTagPose,
                        inputs.tagId,
                        inputs.hasTargets,
                        horizontalDistanceToTargetMeters));
    }

    @AutoLogOutput(key = "Vision/Horizontal Distance to Target")
    public double getHorizontalDistanceToTargetMeters() {
        return horizontalDistanceToTargetMeters;
    }

    @AutoLogOutput(key = "Vision/Has Targets")
    public boolean hasTargets() {
        return getPitchRadians() != 0.00;
        //        return inputs.hasTargets;
    }

    @AutoLogOutput(key = "Vision/Yaw (tx)")
    public double getYawRadians() {
        return inputs.horizontalAngleRadians;
    }

    @AutoLogOutput(key = "Vision/Pitch (ty)")
    public double getPitchRadians() {
        return inputs.verticalAngleRadians;
    }

    public void setLeds(boolean on) {
        visionIO.setLeds(on);
    }
}
