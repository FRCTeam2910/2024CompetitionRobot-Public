package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolation {
    InterpolatingDoubleTreeMap distanceToShooterPitchForSpeaker = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap distanceToPitchForFeedShot = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap distanceToShooterRPMForFeedShot = new InterpolatingDoubleTreeMap();
    private final OperatorDashboard dashboard;

    public ShooterInterpolation(OperatorDashboard dashboard) {
        this.dashboard = dashboard;
        //        distanceToShooterPitch.put(1.68, 56.02);
        //        distanceToShooterPitch.put(2.12, 41.49);
        //        distanceToShooterPitch.put(2.64, 35.10);
        //        distanceToShooterPitch.put(3.08, 32.28);
        //        distanceToShooterPitch.put(3.51, 29.50);
        //        distanceToShooterPitch.put(4.01, 27.38);
        //        distanceToShooterPitch.put(4.62, 25.38);
        //        distanceToShooterPitch.put(4.94, 24.195);
        //        distanceToShooterPitch.put(5.47, 23.37);
        //        distanceToShooterPitch.put(5.92, 22.59);
        //        distanceToShooterPitch.put(6.60, 22.27);
        //        distanceToShooterPitch.put(1.50, 50.22);
        //        distanceToShooterPitch.put(2.00, 42.62);
        //        distanceToShooterPitch.put(2.51, 37.47);
        //        distanceToShooterPitch.put(3.00, 34.25);
        //        distanceToShooterPitch.put(3.51, 31.10);
        //        distanceToShooterPitch.put(4.00, 28.83);
        //        distanceToShooterPitch.put(4.50, 27.29);
        //        distanceToShooterPitch.put(5.01, 26.03);
        //        distanceToShooterPitch.put(5.44, 25.30);
        //        distanceToShooterPitch.put(6.00, 25.00);
        //        distanceToShooterPitch.put(6.52, 24.70);
        //        distanceToShooterPitch.put(1.0, 56.00);
        //        distanceToShooterPitch.put(1.5, 50.20);
        //        distanceToShooterPitch.put(2.0, 46.40);
        //        distanceToShooterPitch.put(2.5, 41.68);
        //        distanceToShooterPitch.put(3.0, 37.65);
        //        distanceToShooterPitch.put(3.5, 35.25);
        //        distanceToShooterPitch.put(4.0, 32.69);
        //        distanceToShooterPitch.put(4.5, 30.83);
        //        distanceToShooterPitch.put(5.0, 29.10);
        //        distanceToShooterPitch.put(5.5, 27.76);
        //        distanceToShooterPitch.put(6.0, 26.64);
        //        distanceToShooterPitch.put(6.5, 26.00);
        //        distanceToShooterPitch.put(7.0, 25.40);
        //        distanceToShooterPitch.put(7.5, 24.80);
        //        distanceToShooterPitch.put(8.0, 24.30);

        distanceToShooterPitchForSpeaker.put(1.0, 57.80 + 4.0);
        distanceToShooterPitchForSpeaker.put(1.5, 49.00 + 2.5);
        distanceToShooterPitchForSpeaker.put(2.0, 42.75);
        distanceToShooterPitchForSpeaker.put(2.5, 36.70);
        distanceToShooterPitchForSpeaker.put(3.0, 34.00);
        distanceToShooterPitchForSpeaker.put(3.5, 31.20);
        distanceToShooterPitchForSpeaker.put(4.0, 29.50);
        distanceToShooterPitchForSpeaker.put(4.5, 27.50);
        distanceToShooterPitchForSpeaker.put(5.0, 26.00);
        distanceToShooterPitchForSpeaker.put(5.5, 24.85);
        distanceToShooterPitchForSpeaker.put(6.0, 23.60);
        distanceToShooterPitchForSpeaker.put(6.5, 22.80);

        distanceToPitchForFeedShot.put(1.0, 55.0 + 1.0);
        distanceToPitchForFeedShot.put(2.0, 55.0 + 1.0);
        distanceToPitchForFeedShot.put(3.0, 55.0 + 1.0);
        distanceToPitchForFeedShot.put(4.0, 55.0 + 1.0);
        distanceToPitchForFeedShot.put(7.0, 55.0 + 1.0);
        distanceToPitchForFeedShot.put(8.0, 55.0 + 1.0);
        distanceToPitchForFeedShot.put(9.5, 50.0 + 1.0);
        distanceToPitchForFeedShot.put(11.5, 45.0 + 1.0);

        distanceToShooterRPMForFeedShot.put(1.0, 2000.0 * 0.86);
        distanceToShooterRPMForFeedShot.put(2.0, 3000.0 * 0.86);
        distanceToShooterRPMForFeedShot.put(3.0, 4500.0 * 0.86);
        distanceToShooterRPMForFeedShot.put(4.0, 5200.0 * 0.86);
        distanceToShooterRPMForFeedShot.put(7.0, 6000.0 * 0.86);
        distanceToShooterRPMForFeedShot.put(8.0, 6800.0 * 0.86);
        distanceToShooterRPMForFeedShot.put(9.5, 7200.0 * 0.86);
        distanceToShooterRPMForFeedShot.put(11.5, 8000.0 * 0.86);
    }

    public Rotation2d getPitch(double distance) {
        return Rotation2d.fromDegrees(
                distanceToShooterPitchForSpeaker.get(Double.valueOf(distance)) + dashboard.getPitchOffset());
    }

    public Rotation2d getFeedShotPitch(double distance) {
        return Rotation2d.fromDegrees(distanceToPitchForFeedShot.get(Double.valueOf(distance)));
    }

    public double getFeedShotRPM(double distance) {
        return distanceToShooterRPMForFeedShot.get(Double.valueOf(distance)) + dashboard.getFeedRPMOffset();
    }
}
