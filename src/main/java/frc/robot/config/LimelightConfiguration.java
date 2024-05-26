package frc.robot.config;

@SuppressWarnings("InconsistentCapitalization")
public class LimelightConfiguration {
    public String Name = "limelight";
    public double LimelightMountingRollRadians = 0;
    public double LimelightMountingYawRadians = 0;
    public double LimelightMountingPitchRadians = 0;
    public double LimelightHeightOffsetMeters = 0;
    public double LimelightLengthOffsetMeters = 0;
    public double LimelightWidthOffsetMeters = 0;

    public LimelightConfiguration withName(String name) {
        this.Name = name;
        return this;
    }

    public LimelightConfiguration withMountingRoll(double rollRadians) {
        this.LimelightMountingRollRadians = rollRadians;
        return this;
    }

    public LimelightConfiguration withMountingYaw(double yawRadians) {
        this.LimelightMountingYawRadians = yawRadians;
        return this;
    }

    public LimelightConfiguration withMountingPitch(double pitchRadians) {
        this.LimelightMountingPitchRadians = pitchRadians;
        return this;
    }

    public LimelightConfiguration withHeightOffset(double heightOffsetMeters) {
        this.LimelightHeightOffsetMeters = heightOffsetMeters;
        return this;
    }

    public LimelightConfiguration withLengthOffset(double lengthOffsetMeters) {
        this.LimelightLengthOffsetMeters = lengthOffsetMeters;
        return this;
    }

    public LimelightConfiguration withWidthOffset(double widthOffsetMeters) {
        this.LimelightWidthOffsetMeters = widthOffsetMeters;
        return this;
    }
}
